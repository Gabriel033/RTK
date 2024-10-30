import serial
import time
from conf import RTK_OPTIONS
count = 0

# Función para obtener los datos GPS desde un dispositivo RTK conectado a un puerto serial
def obtener_datos_gps_rtk(puerto, velocidad_baudios):
    global count
    try:
        # Inicializar variables antes del bloque try
        ser = None
        buffer_serial = ''

        # Conectar al dispositivo RTK usando el puerto serial
        ser = serial.Serial(puerto, velocidad_baudios, timeout=0)
        if count == 0:
            print(f"Conectado al dispositivo RTK en {puerto} a {velocidad_baudios} baudios")
            count = 1

        while True:
            # Verificar cuántos bytes hay en espera en el buffer
            datos_disponibles = ser.in_waiting
            if datos_disponibles > 0:
                # Leer todos los datos disponibles
                datos_leidos = ser.read(datos_disponibles).decode('utf-8', errors='replace')
                buffer_serial += datos_leidos

                # Procesar las líneas completas
                while '\n' in buffer_serial:
                    linea, buffer_serial = buffer_serial.split('\n', 1)
                    linea = linea.strip()

                    # Comprobar si la línea contiene una frase NMEA de interés
                    if linea.startswith('$GNGGA') or linea.startswith('$GPGGA') or linea.startswith('$GNRMC'):
                        # Dividir la línea en sus componentes
                        datos = linea.split(',')

                        # Asegurarse de que la frase tiene suficientes campos y datos válidos
                        if len(datos) > 5:
                            if linea.startswith('$GNRMC'):
                                # Procesar sentencia RMC
                                if datos[3] and datos[5]:
                                    latitud = convertir_latitud(datos[3], datos[4])
                                    longitud = convertir_longitud(datos[5], datos[6])
                                    if latitud is not None and longitud is not None:
                                        yield (latitud, longitud)
                            else:
                                # Procesar sentencia GGA
                                if len(datos) > 9 and datos[2] and datos[4]:
                                    latitud = convertir_latitud(datos[2], datos[3])
                                    longitud = convertir_longitud(datos[4], datos[5])
                                    if latitud is not None and longitud is not None:
                                        yield (latitud, longitud)
            else:
                #time.sleep(0.001)
                continue
    except serial.SerialException as e:
        print(f"Error al abrir el puerto {puerto}: {e}")
    except Exception as e:
        print(f"Error al obtener datos GPS: {e}")
    finally:
        if ser:
            ser.close()

# Función para convertir las coordenadas de latitud en formato NMEA a formato decimal
def convertir_latitud(latitud_nmea, direccion):
    try:
        if not latitud_nmea:
            return None
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
        if not longitud_nmea:
            return None
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
    puerto = RTK_OPTIONS.get('port')
    baud_rate = RTK_OPTIONS.get('baud_rate')
    for lat, lon in obtener_datos_gps_rtk(puerto, baud_rate):
        print(f"Latitud: {lat}, Longitud: {lon}")
