import osmnx as ox

from RTK_Real_Data import obtener_datos_gps_rtk
from conf import RTK_OPTIONS

# Coordenadas actuales del RTK
try:
    current_position = next(obtener_datos_gps_rtk(RTK_OPTIONS.get('port'), RTK_OPTIONS.get('baud_rate')))
    print("Dispositivo RTK detectado, se inicia el guardado del mapa")
except:
    print("Dispositivo RTK no detectado, guardando mapa con coordenadas manuales")
    current_position = [19.017360, -98.242064]

# Definir un radio de 1 kilómetro alrededor de la ubicación del RTK
dist = 2000  # Distancia en metros

# Descargar el grafo para la red vial en un radio de 1 kilómetro
G = ox.graph_from_point(current_position, dist=dist, network_type='all', simplify=False)
edges = ox.graph_to_gdfs(G, nodes=False)
edges = edges[edges['highway'].isin(['residential', 'service', 'unclassified', 'tertiary', 'secondary', 'primary', 'trunk', 'motorway'])]
G = ox.graph_from_gdfs(ox.graph_to_gdfs(G, nodes=True)[0], edges)

# Guardar el grafo en un archivo para uso offline
ox.save_graphml(G, filepath="templates/grafo_rtk.graphml")
print("Mapa guardado!")

ox.plot_graph(G)
