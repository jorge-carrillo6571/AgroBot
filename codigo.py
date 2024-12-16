import socket
import rospy
from std_msgs.msg import Float32, Header
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

# Configuración del socket
HOST = '0.0.0.0'  # Escucha en todas las interfaces de red disponibles
PORT = 5000       # Puerto de escucha

pos = [0,0]
def odom_callback(msg):
    global pos
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    pos = [pos_x, pos_y]

def create_marker(marker_id, x, y, tempe, humid):
    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "map"     #desired frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "marker_array"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = -x
    marker.pose.position.y = -y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.text = f"{tempe}°C, {humid}%"

    if tempe > 32: 
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    
    if tempe <= 32: 
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

    return marker

def start_server():
    global pos
    # Inicializa el nodo de ROS
    rospy.init_node('socket_listener_node', anonymous=True)
    # Define los publicadores para los tópicos 'temperature' y 'humidity'
    # temp_pub = rospy.Publisher('temperature', Float32, queue_size=10)
    # hum_pub = rospy.Publisher('humidity', Float32, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    markerArray_pub_Temp = rospy.Publisher('/marker_array_topic_temp', Marker, queue_size=10)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        rospy.loginfo(f"Servidor escuchando en {HOST}:{PORT}")
        markerId = 0

        while not rospy.is_shutdown():
            conn, addr = s.accept()
            with conn:
                rospy.loginfo(f"Conexión establecida con {addr}")
                data = conn.recv(1024)
                if not data:
                    break
                try:
                    # Convierte los datos recibidos en una lista de valores float
                    received_values = data.decode('utf-8').split(',')
                    temperature = float(received_values[0])
                    humidity = float(received_values[1])
                    rospy.loginfo(f"Temperatura recibida: {temperature}")
                    rospy.loginfo(f"Humedad recibida: {humidity}")
                    # # Publica los valores recibidos en los respectivos tópicos de ROS
                    # temp_pub.publish(temperature)
                    # hum_pub.publish(humidity)
                    marker = create_marker(markerId, pos[0], pos[1], temperature, humidity)
                    markerArray_pub_Temp.publish(marker)
                    #rospy.sleep(2)
                    # marker_array.markers.append(marker)
                    markerId += 1

                except (ValueError, IndexError) as e:
                    rospy.logwarn(f"No se pudo procesar los datos recibidos: {e}")

if __name__ == '__main__':
    try:
        start_server()
    except rospy.ROSInterruptException:
        pass
