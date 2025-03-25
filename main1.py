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

        self.grab_button = QtWidgets.QPushButton('Захват', self)
        self.grab_button.clicked.connect(self.grab_object)

        # Ползунки для задания углов поворота
        self.angle_slider_x = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.angle_slider_x.setRange(0, 360)
        self.angle_slider_x.setValue(180)  # Начальное значение

        self.angle_slider_y = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.angle_slider_y.setRange(0, 360)
        self.angle_slider_y.setValue(180)  # Начальное значение

        self.angle_slider_z = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.angle_slider_z.setRange(0, 360)
        self.angle_slider_z.setValue(180)  # Начальное значение

        # Макеты
        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.start_button)
        self.layout.addWidget(self.stop_button)
        self.layout.addWidget(self.grab_button)

        # Добавление ползунков
        self.layout.addWidget(QtWidgets.QLabel('Угол поворота X:'))
        self.layout.addWidget(self.angle_slider_x)
        self.layout.addWidget(QtWidgets.QLabel('Угол поворота Y:'))
        self.layout.addWidget(self.angle_slider_y)
        self.layout.addWidget(QtWidgets.QLabel('Угол поворота Z:'))
        self.layout.addWidget(self.angle_slider_z)

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
        self.move_robot()

    def stop_robot(self):
        rospy.loginfo("Остановка робота...")
        self.send_command("stop")  # Отправка команды на остановку робота

    def grab_object(self):
        rospy.loginfo("Захват объекта...")
        self.send_command("grab")  # Отправка команды на захват объекта

    def move_robot(self):
        x_angle = self.angle_slider_x.value()
        y_angle = self.angle_slider_y.value()
        z_angle = self.angle_slider_z.value()
        command = f"move {x_angle} {y_angle} {z_angle}"
        self.send_command(command)

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
