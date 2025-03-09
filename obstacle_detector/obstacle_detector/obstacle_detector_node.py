import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        # Suscribirse al tópico /scan para obtener datos del LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # Evitar advertencia de variable no utilizada
        self.detected_obstacles = []  # Lista para almacenar obstáculos detectados
        self.obstacle_count = 0  # Contador de obstáculos

    def scan_callback(self, msg):
        ranges = msg.ranges  # Datos de distancia del LIDAR
        angle_min = msg.angle_min  # Ángulo mínimo del LIDAR
        angle_increment = msg.angle_increment  # Incremento angular entre mediciones

        # Recorrer todas las mediciones del LIDAR
        for i, distance in enumerate(ranges):
            if distance < 3.5:  # Si hay un obstáculo dentro del rango del LIDAR
                angle = angle_min + i * angle_increment  # Calcular el ángulo actual
                x = distance * math.cos(angle)  # Posición en X (coordenadas cartesianas)
                y = distance * math.sin(angle)  # Posición en Y (coordenadas cartesianas)

                # Verificar si el obstáculo ya fue detectado
                if not self.is_obstacle_detected(x, y):
                    # Clasificar el obstáculo basado en la distancia
                    if distance < 1.0:  # Obstáculo esférico
                        size = 0.2  # Diámetro en metros
                        obstacle_type = 'esférico'
                    else:  # Obstáculo en forma de caja
                        size = 0.3  # Tamaño en metros
                        obstacle_type = 'caja'

                    # Registrar el obstáculo
                    self.detected_obstacles.append((obstacle_type, x, y, size))
                    self.obstacle_count += 1  # Incrementar el contador de obstáculos
                    self.get_logger().info(f'Obstáculo {self.obstacle_count}: {obstacle_type} detectado en ({x:.2f}, {y:.2f}), tamaño: {size:.2f}m')

    def is_obstacle_detected(self, x, y):
        # Verificar si el obstáculo ya está en la lista de detectados
        for obstacle in self.detected_obstacles:
            ox, oy = obstacle[1], obstacle[2]  # Posición del obstáculo ya registrado
            # Si la distancia entre el obstáculo detectado y uno ya registrado es menor que un umbral, se considera el mismo
            if math.sqrt((x - ox)**2 + (y - oy)**2) < 0.1:  # Umbral de 0.1 metros
                return True
        return False

    def save_results(self):
        # Guardar los obstáculos detectados en un archivo de texto
        with open('obstacles.txt', 'w') as f:
            f.write(f'Total de obstáculos detectados: {self.obstacle_count}\n')
            for i, obstacle in enumerate(self.detected_obstacles):
                f.write(f'Obstáculo {i + 1}: {obstacle[0]}, ({obstacle[1]:.2f}, {obstacle[2]:.2f}), tamaño: {obstacle[3]:.2f}m\n')

def main(args=None):
    rclpy.init(args=args)  # Inicializar ROS 2
    obstacle_detector = ObstacleDetector()  # Crear una instancia del nodo
    try:
        rclpy.spin(obstacle_detector)  # Mantener el nodo en ejecución
    except KeyboardInterrupt:
        # Guardar los resultados al finalizar la ejecución (Ctrl+C)
        obstacle_detector.save_results()
    finally:
        obstacle_detector.destroy_node()  # Destruir el nodo
        rclpy.shutdown()  # Apagar ROS 2

if __name__ == '__main__':
    main()  # Ejecutar la función main
