import requests
from flask import Flask, render_template_string, request, jsonify
import folium
import osmnx as ox
import networkx as nx
import numpy as np
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
setpoints = []  # Lista para guardar las coordenadas de los puntos de la ruta

# Cambiar entre coordenadas manuales y RTK
use_manual_coordinates = True  # Cambia a False para usar el RTK
posicion_manual = [19.017360, -98.242064]  # Coordenadas manuales


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


def calcular_setpoints_desde_ruta(route, interval=0.00004):  # Intervalo en grados (aprox. 0.5 metros)
    global setpoints
    setpoints = []

    punto_anterior = np.array(route[0])
    setpoints.append(route[0])  # Agregar el primer punto (origen)

    distancia_acumulada = 0

    # Iterar sobre cada segmento de la ruta
    for i in range(1, len(route)):
        punto_actual = np.array(route[i])
        distancia_segmento = np.linalg.norm(punto_actual - punto_anterior)

        # Generar setpoints en intervalos fijos dentro del segmento
        while distancia_acumulada + distancia_segmento >= interval:
            # Calculamos la fracción donde generar el nuevo punto
            fraccion = (interval - distancia_acumulada) / distancia_segmento
            nuevo_punto = punto_anterior + fraccion * (punto_actual - punto_anterior)

            # Guardamos el nuevo setpoint
            setpoints.append(tuple(nuevo_punto.tolist()))
            # print(f"Setpoint generado: {nuevo_punto}")

            # Actualizamos la distancia acumulada y avanzamos al siguiente punto
            distancia_segmento -= interval
            distancia_acumulada = 0  # Reiniciamos la acumulación de la distancia
            punto_anterior = nuevo_punto  # El nuevo punto se convierte en el anterior

        # Acumular el resto de la distancia
        distancia_acumulada += distancia_segmento
        punto_anterior = punto_actual

    # Asegurarse de agregar el último punto (destino)
    if np.linalg.norm(np.array(setpoints[-1]) - np.array(route[-1])) > 0:
        setpoints.append(tuple(route[-1]))
    return setpoints


# Función para calcular y suavizar la ruta
def calcular_ruta(origen, destino):
    global setpoints
    try:
        # Inicializar la variable route en caso de que no se encuentre una ruta válida
        route = None

        # Obtener los nodos más cercanos al origen y al destino
        origen_nodo = ox.distance.nearest_nodes(G, origen[1], origen[0])
        destino_nodo = ox.distance.nearest_nodes(G, destino[1], destino[0])

        # Verificar si existe un camino entre el nodo de origen y el nodo de destino
        if nx.has_path(G, origen_nodo, destino_nodo):
            # Calcular la ruta más corta
            route = nx.shortest_path(G, origen_nodo, destino_nodo, weight='length')
            smoothed_route = []

            # Recorrer la ruta y almacenar las coordenadas suavizadas
            for i in range(1, len(route)):
                current_node = route[i]
                smoothed_route.append((G.nodes[current_node]['y'], G.nodes[current_node]['x']))

            # Verificar si el punto de partida no es el primer punto de la ruta
            if np.linalg.norm(np.array(current_position) - np.array(smoothed_route[0])) > 0.0001:
                # Agregar una conexión directa entre current_position y el inicio de la ruta
                smoothed_route.insert(0, current_position)

            # Generar setpoints desde el primer punto
            setpoints = calcular_setpoints_desde_ruta(smoothed_route)

            # Verificar si el punto final está cerca del destino, si no, agregar una línea para conectar
            if np.linalg.norm(np.array(destino) - np.array(smoothed_route[-1])) > 0.0001:
                # Si la distancia es menor a 0.05, conectar al destino
                if np.linalg.norm(np.array(smoothed_route[-1]) - np.array(destino)) < 0.05:
                    smoothed_route.append(destino)

            return smoothed_route

        # En caso de que no haya un camino entre el origen y el destino
        else:
            print("No se encontró un camino para llegar al destino.")  # Para depuración
            return {"error": "No se encontró un camino para llegar a su destino."}

    except Exception as e:
        # Capturar cualquier excepción que ocurra y devolver el error
        print("Error al calcular la ruta:", str(e))  # Para depuración
        return {"error": str(e)}


# Función para vaciar los setpoints al llegar al destino
def actualizar_posicion():
    global current_position, is_moving, route, destination, setpoints
    is_moving = True
    while True:
        # Usar coordenadas manuales o las del RTK
        if use_manual_coordinates:
            current_position = posicion_manual  # Coordenadas manuales
        else:
            for lat, lon in obtener_datos_gps_rtk('COM4', 9600):
                current_position = [lat, lon]
                break

        if route and destination:
            if np.linalg.norm(np.array(current_position) - np.array(destination)) < 0.0001:
                route = []
                destination = None
                ruta_confirmada = False
                setpoints = []  # Vaciar los setpoints al llegar al destino
                notify_arrival()  # Mostrar alerta de llegada al destino
                break
        time.sleep(0.5)
    is_moving = False


# Función para enviar la ruta actualizada al frontend
def enviar_ruta_actualizada():
    global route
    if route:
        route_coords = [{"lat": coord[0], "lon": coord[1]} for coord in route]
        requests.post('http://127.0.0.1:5000/update_route', json={"route": route_coords})


@app.route('/get_position', methods=['GET'])
def get_position():
    return jsonify({"lat": current_position[0], "lon": current_position[1]})


@app.route('/update_route', methods=['POST'])
def update_route():
    global route
    route = request.json.get('route')
    return jsonify({"status": "success"})


@app.route('/notify_arrival', methods=['POST'])
def notify_arrival():
    return jsonify({"message": "Has llegado a tu destino"})


@app.route('/')
def mostrar_mapa():
    global ruta_confirmada, destination, setpoints, route
    m = folium.Map(location=current_position, zoom_start=14)
    folium.Marker(current_position, tooltip="Posición Actual", icon=folium.Icon(color='blue')).add_to(m)

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
            style="position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%); display: {'block' if ruta_confirmada else 'none'}; font-size: 40px; padding: 40px 60px;"
            onclick="cambiarRuta()">CAMBIAR RUTA
        </button>
        <button id="centrarBtn" 
            style="position: absolute; bottom: 20px; left: 10px; font-size: 40px; padding: 40px 60px;"
            onclick="centrarEnVehiculo()">CENTRAR EN EL VEHÍCULO
        </button>
        <script>
            var rutaConfirmada = {str(ruta_confirmada).lower()};
            var seguirVehiculo = false;  // Flag para controlar si seguimos o no al vehículo

            var map = L.map('mapid').setView([{current_position[0]}, {current_position[1]}], 17);

            L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                maxZoom: 19,
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }}).addTo(map);

            var marker = L.marker([{current_position[0]}, {current_position[1]}]).addTo(map).bindTooltip("Posición Actual").openTooltip();
            var routePolyline = L.polyline([], {{color: 'blue', weight: 5}}).addTo(map);
            var setpointMarkers = [];

            // Función para centrar en el vehículo y seguirlo
            function centrarEnVehiculo() {{
                seguirVehiculo = true;
                map.setView(marker.getLatLng(), map.getZoom());
            }}

            // Evento para deshabilitar seguimiento del vehículo si el usuario mueve el mapa
            map.on('movestart', function() {{
                seguirVehiculo = false;
            }});

            // Función para actualizar la posición del marcador
            function updatePosition() {{
                fetch('/get_position')
                    .then(response => response.json())
                    .then(data => {{
                        var newLatLng = new L.LatLng(data.lat, data.lon);
                        marker.setLatLng(newLatLng);
                        if (seguirVehiculo) {{
                            map.setView(newLatLng, map.getZoom());
                        }}
                    }});
            }}
            var isUpdating = false;
            // Función para actualizar la ruta y los setpoints en el mapa
            function updateRoute() {{
                if (!isUpdating) return;
                fetch('/get_route')
                    .then(response => response.json())
                    .then(data => {{
                        var routeCoords = data.route.map(function(coord) {{
                            return [coord.lat, coord.lon];
                        }});
                        routePolyline.setLatLngs(routeCoords);

                        // Limpiar los setpoints anteriores
                        setpointMarkers.forEach(marker => map.removeLayer(marker));
                        setpointMarkers = [];

                        // Dibujar los nuevos setpoints
                        data.setpoints.forEach(function(sp) {{
                            var marker = L.circleMarker([sp.lat, sp.lon], {{
                                radius: 2,           // Tamaño pequeño
                                color: "red",         // Color del borde
                                weight: 2,            // Grosor del borde, menor valor = borde más delgado
                                opacity: 0.6,       // Menor opacidad del borde
                                fill: true,           // Que siga teniendo relleno
                                fillColor: "red",     // Color del relleno
                                fillOpacity: 0.3      // Relleno menos opaco para que no se vea tan sólido
                            }});
                            setpointMarkers.push(marker);
                            marker.addTo(map);
                        }});
                    }})
                    .catch(error => console.error('Error al actualizar la ruta:', error));
            }}

            // Actualizar la posición, la ruta y los setpoints cada segundo
            setInterval(updatePosition, 1000);
            setInterval(updateRoute, 1000);

            var destinationMarker;

            // Función para limpiar el mapa
            function clearMap() {{
                isUpdating = false;

                if (routePolyline) {{
                    map.removeLayer(routePolyline);
                }}
                if (destinationMarker) {{
                    map.removeLayer(destinationMarker);
                }}
                // Limpiar los setpoints del mapa
                if (setpointMarkers && setpointMarkers.length > 0) {{
                    setpointMarkers.forEach(function(marker) {{
                        map.removeLayer(marker);  // Eliminar cada marcador de setpoints
                    }});
                    console.log("Setpoints eliminados:", setpointMarkers.length);
                    setpointMarkers = [];  // Vaciar el array de setpoints después de eliminarlos
                }} else {{
                    console.log("No hay setpoints para eliminar.");
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
                routePolyline = L.polyline(routeCoords, {{color: 'blue', weight: 5}}).addTo(map);  // Línea punteada
                isUpdating = true;
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
    global route
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    destination_temp = [lat, lon]
    route_coords = calcular_ruta(current_position, destination_temp)
    if isinstance(route_coords, dict) and "error" in route_coords:
        return jsonify(route_coords)
    else:
        route = route_coords  # Almacenar la ruta correctamente
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
    global route, setpoints
    if route:
        route_coords = [{"lat": coord[0], "lon": coord[1]} for coord in route]
        setpoints_coords = [{"lat": sp[0], "lon": sp[1]} for sp in setpoints]  # Añadir los setpoints
        return jsonify({"route": route_coords, "setpoints": setpoints_coords})  # Devuelve ambos
    else:
        return jsonify({"error": "No hay ruta disponible"})


def iniciar_mapa():
    Thread(target=actualizar_posicion).start()
    app.run(debug=True, use_reloader=False)


if __name__ == "__main__":
    print("Starting program...")

    if use_manual_coordinates:
        current_position = posicion_manual  # Coordenadas manuales
    else:
        current_position = next(obtener_datos_gps_rtk('COM4', 9600))

    # Preguntar si se quiere iniciar en modo offline
    user_input = input("¿Deseas iniciar en modo offline? (s/n): ").strip().lower()
    offline_mode = user_input == 's'
    inicializar_grafo()

    iniciar_mapa()