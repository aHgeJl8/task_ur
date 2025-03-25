import sys
import rospy
from std_msgs.msg import String
from PyQt5 import QtWidgets, QtGui, QtCore
import cv2

class RobotControlApp(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        
        # Инициализация ROS
        rospy.init_node('robot_control_node', anonymous=True)
        
        # Подписка на топик для получения состояния робота
        self.state_subscriber = rospy.Subscriber('robot_state', String, self.state_callback)
        
        # Инициализация видеопотока
        self.cap = cv2.VideoCapture(0)  # Замените 0 на нужный индекс камеры

    def initUI(self):
        self.setWindowTitle('Управление роботизированной ячейкой')
        self.setGeometry(100, 100, 800, 600)

        # Кнопки управления
        self.start_button = QtWidgets.QPushButton('Запуск', self)
        self.start_button.clicked.connect(self.start_robot)

        self.stop_button = QtWidgets.QPushButton('Стоп', self)
        self.stop_button.clicked.connect(self.stop_robot)

        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.start_button)
        self.layout.addWidget(self.stop_button)

        self.setLayout(self.layout)

        # Метка для отображения видео
        self.video_label = QtWidgets.QLabel(self)
        self.layout.addWidget(self.video_label)

        # Запуск таймера для обновления видео
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_video)
        self.timer.start(30)  # Обновление каждые 30 мс

    def start_robot(self):
        rospy.loginfo("Запуск робота...")
        self.send_command("start")  # Отправка команды на запуск робота

    def stop_robot(self):
        rospy.loginfo("Остановка робота...")
        self.send_command("stop")  # Отправка команды на остановку робота

    def send_command(self, command):
        # Публикация команды в топик
        command_publisher = rospy.Publisher('robot_command', String, queue_size=10)
        command_publisher.publish(command)

    def state_callback(self, msg):
        # Обработка состояния робота
        rospy.loginfo("Состояние робота: %s", msg.data)

    def update_video(self):
        ret, frame = self.cap.read()
        if ret:
            # Конвертируем BGR в RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            convert_to_Qt_format = QtGui.QImage(rgb_frame.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            self.video_label.setPixmap(QtGui.QPixmap.fromImage(convert_to_Qt_format))

    def closeEvent(self, event):
        self.cap.release()  # Освобождаем видеопоток
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = RobotControlApp()
    window.show()
    sys.exit(app.exec_())
