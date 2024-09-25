import requests
from flask import Flask, render_template_string, request, jsonify
import folium
import osmnx as ox
import networkx as nx
import numpy as np
import scipy
import time
from threading import Thread
from RTK_Real_Data import obtener_datos_gps_rtk  # Importa la función de RTK

app = Flask(__name__)
offline_mode = None
current_position = None
destination = None
route = None
is_moving = False
ruta_confirmada = False
G = None
n = None  # Distancia en metros entre los puntos de control
setpoints = []  # Lista para guardar las coordenadas de los setpoints


def inicializar_grafo():
    global G
    if offline_mode:
        # Cargar el grafo desde el archivo
        G = ox.load_graphml(filepath="templates/grafo_rtk.graphml")
        print("Grafo cargado desde el archivo local.")
    else:
        # Descargar el grafo desde OSM en tiempo real
        G = ox.graph_from_point(current_position, dist=500, network_type='all', simplify=False)
        edges = ox.graph_to_gdfs(G, nodes=False)
        edges = edges[edges['highway'].isin(
            ['residential', 'service', 'unclassified', 'tertiary', 'secondary', 'primary', 'trunk', 'motorway'])]
        G = ox.graph_from_gdfs(ox.graph_to_gdfs(G, nodes=True)[0], edges)
        print("Grafo descargado desde OpenStreetMap.")


# Función para dividir la ruta en fragmentos de n metros
def calcular_setpoints(route, n):
    global setpoints
    setpoints = []
    distancia_acumulada = 0

    for i in range(1, len(route)):
        punto_anterior = np.array(route[i - 1])
        punto_actual = np.array(route[i])
        distancia_segmento = np.linalg.norm(punto_actual - punto_anterior)

        while distancia_acumulada + distancia_segmento >= n:
            fraccion = (n - distancia_acumulada) / distancia_segmento
            nuevo_punto = punto_anterior + fraccion * (punto_actual - punto_anterior)
            setpoints.append(nuevo_punto.tolist())
            distancia_acumulada = 0
            punto_anterior = nuevo_punto
            distancia_segmento = np.linalg.norm(punto_actual - punto_anterior)

        distancia_acumulada += distancia_segmento

    # Agregar el último punto si está cerca del destino
    if np.linalg.norm(np.array(setpoints[-1]) - np.array(route[-1])) > n:
        setpoints.append(route[-1])

    print("Setpoints:", setpoints)


# Función para dibujar la ruta
def calcular_ruta(origen, destino):
    try:
        origen_nodo = ox.distance.nearest_nodes(G, origen[1], origen[0])
        destino_nodo = ox.distance.nearest_nodes(G, destino[1], destino[0])

        if nx.has_path(G, origen_nodo, destino_nodo):
            route = nx.shortest_path(G, origen_nodo, destino_nodo, weight='length')
            smoothed_route = []

            for i in range(1, len(route) - 1):
                prev_node = route[i - 1]
                current_node = route[i]
                next_node = route[i + 1]

                vector1 = np.array([G.nodes[current_node]['x'] - G.nodes[prev_node]['x'],
                                    G.nodes[current_node]['y'] - G.nodes[prev_node]['y']])
                vector2 = np.array([G.nodes[next_node]['x'] - G.nodes[current_node]['x'],
                                    G.nodes[next_node]['y'] - G.nodes[current_node]['y']])
                cos_angle = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
                angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

                if np.degrees(angle) < 150:
                    if len(smoothed_route) == 0:
                        smoothed_route.append((G.nodes[prev_node]['y'], G.nodes[prev_node]['x']))

                    latitudes, longitudes = zip(
                        (G.nodes[prev_node]['y'], G.nodes[prev_node]['x']),
                        (G.nodes[current_node]['y'], G.nodes[current_node]['x']),
                        (G.nodes[next_node]['y'], G.nodes[current_node]['x'])
                    )

                    if len(latitudes) >= 4:
                        tck, u = scipy.interpolate.splprep([latitudes, longitudes], s=0)
                        interpolated_coords = scipy.interpolate.splev(np.linspace(0, 1, 10), tck)
                        smoothed_route.extend(zip(interpolated_coords[0], interpolated_coords[1]))
                    else:
                        smoothed_route.append((G.nodes[current_node]['y'], G.nodes[current_node]['x']))
                else:
                    smoothed_route.append((G.nodes[current_node]['y'], G.nodes[current_node]['x']))

            smoothed_route.append((G.nodes[route[-1]]['y'], G.nodes[route[-1]]['x']))
            calcular_setpoints(smoothed_route, n)
            return smoothed_route

        else:
            return {
                "error": "No se encontró un camino para llegar a su destino, ya que se encuentra fuera de la vialidad o el punto de destino no se conecta con el camino. Colóquese en la avenida vehicular y seleccione su punto de destino."}

    except Exception as e:
        return {"error": str(e)}


# Función para actualizar la posición en el mapa con datos reales del RTK
def actualizar_posicion():
    global current_position, is_moving, route, destination
    is_moving = True
    for lat, lon in obtener_datos_gps_rtk('COM4', 9600):  # Ajusta el puerto y la velocidad según tu configuración
        current_position = [lat, lon]

        if route and destination:
            if np.linalg.norm(np.array(current_position) - np.array(destination)) < 0.0001:
                route = []
                destination = None
                ruta_confirmada = False
                # Mostrar alerta de llegada al destino
                notify_arrival()
                break
            else:
                route.pop(0)  # Eliminar el primer punto de la ruta ya recorrida
                enviar_ruta_actualizada()

        time.sleep(1)
    is_moving = False


def enviar_ruta_actualizada():
    global route
    route_coords = [{"lat": coord[0], "lon": coord[1]} for coord in route]
    requests.post('/update_route', json={"route": route_coords})


@app.route('/get_position', methods=['GET'])
def get_position():
    return jsonify({"lat": current_position[0], "lon": current_position[1]})


@app.route('/notify_arrival', methods=['POST'])
def notify_arrival():
    return jsonify({"message": "Has llegado a tu destino"})


# Modificación en el template HTML para incluir la alerta
@app.route('/')
def mostrar_mapa():
    global ruta_confirmada, destination
    m = folium.Map(location=current_position, zoom_start=14)
    folium.Marker(current_position, tooltip="Posición Actual", icon=folium.Icon(color='blue')).add_to(m)

    if destination:
        folium.Marker(destination, tooltip="Destino", icon=folium.Icon(color='red')).add_to(m)
        if route:
            folium.PolyLine(route, color="blue", weight=5).add_to(m)
            cambiar_ruta_visible = True
    else:
        cambiar_ruta_visible = False

    map_html = m._repr_html_()
    return render_template_string(f'''
    <html>
    <head>
        <title>Mapa RTK</title>
        <meta charset="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>

        <style>
            #mapid {{
                border: 8px solid black; 
                height: 600px;
            }}
            body {{
                background-color: #f0f0f0;
                margin: 0;
            }}
            .map-container {{
                padding: 20px;
                background-color: white; 
            }}
        </style>

    </head>
    <body>
        <div id="mapid" style="height: 700px;"></div>
        <button id="cambiarRutaBtn" 
            style="position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%); display: {'block' if cambiar_ruta_visible else 'none'}; font-size: 40px; padding: 40px 60px;"
            onclick="cambiarRuta()">CAMBIAR RUTA
        </button>
        <script>
            var rutaConfirmada = {str(ruta_confirmada).lower()};
            var map = L.map('mapid').setView([{current_position[0]}, {current_position[1]}], 17);

            L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                maxZoom: 19,
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }}).addTo(map);

            var marker = L.marker([{current_position[0]}, {current_position[1]}]).addTo(map).bindTooltip("Posición Actual").openTooltip();
            var routePolyline = L.polyline([], {{color: 'blue', weight: 5}}).addTo(map);

            // Función para actualizar la posición del marcador
            function updatePosition() {{
                fetch('/get_position')
                    .then(response => response.json())
                    .then(data => {{
                        var newLatLng = new L.LatLng(data.lat, data.lon);
                        marker.setLatLng(newLatLng);
                        map.setView(newLatLng, map.getZoom());
                    }});
            }}

            // Función para actualizar la ruta en el mapa
            function updateRoute() {{
                fetch('/get_route')
                    .then(response => response.json())
                    .then(data => {{
                        var routeCoords = data.route.map(function(coord) {{
                            return [coord.lat, coord.lon];
                        }});
                        routePolyline.setLatLngs(routeCoords);
                    }});
            }}

            // Actualizar la posición y la ruta cada segundo
            setInterval(updatePosition, 1000);
            setInterval(updateRoute, 1000);

            var destinationMarker;

            // Función para limpiar el mapa
            function clearMap() {{
                if (routePolyline) {{
                    map.removeLayer(routePolyline);
                }}
                if (destinationMarker) {{
                    map.removeLayer(destinationMarker);
                }}
                document.getElementById('cambiarRutaBtn').style.display = 'none';
                rutaConfirmada = false;
            }}

            // Función para notificar llegada y limpiar el mapa
            function notify_arrival() {{
                fetch('/notify_arrival', {{method: 'POST'}})
                .then(response => response.json())
                .then(data => {{
                    alert(data.message);
                    clearMap();  // Limpiar el mapa después de cerrar la alerta
                }});
            }}

            function cambiarRuta() {{
                clearMap();
            }}

            // Función para dibujar la ruta en el mapa
            function drawRouteOnMap(routeCoords) {{
                if (routePolyline) {{
                    map.removeLayer(routePolyline);
                }}
                routePolyline = L.polyline(routeCoords, {{color: 'blue', weight: 5}}).addTo(map);

                setTimeout(() => {{
                    var confirmed = confirm("¿Es correcta la posición de destino?");
                    if (confirmed) {{
                        fetch(`/set_destination?lat=${{routeCoords[routeCoords.length - 1][0]}}&lon=${{routeCoords[routeCoords.length - 1][1]}}`).then(() => {{
                            rutaConfirmada = true;
                            document.getElementById('cambiarRutaBtn').style.display = 'block';
                        }});
                    }} else {{
                        clearMap();  // Si no se confirma, limpiar el mapa
                    }}
                }}, 500);
            }}

            // Manejador de clics en el mapa
            map.on('click', function(e) {{
                if (rutaConfirmada) {{
                    alert("Ya hay una ruta confirmada. Use el botón CAMBIAR RUTA para seleccionar un nuevo destino.");
                    return;
                }}

                if (destinationMarker) {{
                    map.removeLayer(destinationMarker);
                }}
                if (routePolyline) {{
                    map.removeLayer(routePolyline);
                }}

                destinationMarker = L.marker([e.latlng.lat, e.latlng.lng]).addTo(map).bindTooltip("Destino").openTooltip();

                fetch(`/calculate_route?lat=${{e.latlng.lat}}&lon=${{e.latlng.lng}}`)
                    .then(response => response.json())
                    .then(data => {{
                        if (data.error) {{
                            alert(data.error);
                            if (destinationMarker) {{
                                map.removeLayer(destinationMarker);
                            }}
                        }} else {{
                            drawRouteOnMap(data.route);  // Dibuja la ruta y solicita confirmación
                        }}
                    }});
            }});

        </script>
    </body>
    </html>
    ''')


@app.route('/calculate_route')
def calculate_route():
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    destination_temp = [lat, lon]
    route_coords = calcular_ruta(current_position, destination_temp)
    if isinstance(route_coords, dict) and "error" in route_coords:
        return jsonify(route_coords)
    else:
        return jsonify({"route": route_coords})


@app.route('/set_destination')
def set_destination():
    global destination, route, ruta_confirmada
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    destination = [lat, lon]
    ruta_confirmada = True
    route = calcular_ruta(current_position, destination)
    return "Destino actualizado"


@app.route('/clear_route')
def clear_route():
    global destination, route, ruta_confirmada
    destination = None
    route = None
    ruta_confirmada = False
    return "Ruta y destino borrados"


@app.route('/get_route', methods=['GET'])
def get_route():
    return jsonify({"route": [{"lat": coord[0], "lon": coord[1]} for coord in route]})


def iniciar_mapa():
    Thread(target=actualizar_posicion).start()
    app.run(debug=True, use_reloader=False)


if __name__ == "__main__":
    print("Starting program...")
    current_position = next(obtener_datos_gps_rtk('COM4', 9600))

    # Preguntar si se quiere iniciar en modo offline
    user_input = input("¿Deseas iniciar en modo offline? (s/n): ").strip().lower()
    offline_mode = user_input == 's'
    inicializar_grafo()

    # Configurar la distancia n para los setpoints
    n = 10  # Puedes cambiar este valor según sea necesario

    iniciar_mapa()
