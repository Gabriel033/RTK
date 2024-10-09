import serial
import time


# Función para obtener los datos GPS desde un dispositivo RTK conectado a un puerto serial
def obtener_datos_gps_rtk(puerto, velocidad_baudios):
    global ser
    try:
        # Conectar al dispositivo RTK usando el puerto serial
        ser = serial.Serial(puerto, velocidad_baudios, timeout=1)
        print(f"Conectado al dispositivo RTK en {puerto} a {velocidad_baudios} baudios")

        while True:
            # Leer una línea del dispositivo RTK
            linea = ser.readline().decode('utf-8', errors='replace').strip()
            print(linea)

            # Comprobar si la línea contiene una frase GGA de NMEA, que proporciona los datos de posición
            if linea.startswith('$GNGGA') or linea.startswith('$GPGGA'):
                # Dividir la línea en sus componentes
                datos = linea.split(',')

                # Asegurarse de que la frase GGA tiene suficiente longitud y datos válidos
                if len(datos) > 9 and datos[2] and datos[4]:
                    # Extraer la latitud y longitud en formato NMEA
                    latitud = convertir_latitud(datos[2], datos[3])
                    longitud = convertir_longitud(datos[4], datos[5])

                    # Comprobar si los datos de latitud y longitud son válidos
                    if latitud is not None and longitud is not None:
                        # Devolver los datos de latitud y longitud
                        yield (latitud, longitud)
            else:
                # Si no hay datos GGA, esperar brevemente y volver a intentar
                time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Error al abrir el puerto {puerto}: {e}")
    except Exception as e:
        print(f"Error al obtener datos GPS: {e}")
    finally:
        ser.close()


# Función para convertir las coordenadas de latitud en formato NMEA a formato decimal
def convertir_latitud(latitud_nmea, direccion):
    try:
        grados = float(latitud_nmea[:2])
        minutos = float(latitud_nmea[2:]) / 60
        latitud = grados + minutos
        if direccion == 'S':
            latitud = -latitud
        return latitud
    except (ValueError, IndexError) as e:
        print(f"Error al convertir latitud: {e}")
        return None


# Función para convertir las coordenadas de longitud en formato NMEA a formato decimal
def convertir_longitud(longitud_nmea, direccion):
    try:
        grados = float(longitud_nmea[:3])
        minutos = float(longitud_nmea[3:]) / 60
        longitud = grados + minutos
        if direccion == 'W':
            longitud = -longitud
        return longitud
    except (ValueError, IndexError) as e:
        print(f"Error al convertir longitud: {e}")
        return None


if __name__ == "__main__":
    puerto = 'COM3'
    baud_rate = 9600
    for lat, lon in obtener_datos_gps_rtk(puerto, baud_rate):
        print(f"Latitud: {lat}, Longitud: {lon}")