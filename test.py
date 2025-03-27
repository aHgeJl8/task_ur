from graphviz import Digraph

def create_er_diagram(class_name, attributes, primary_key):
    # Создаем новый граф
    dot = Digraph()

    # Добавляем класс как узел
    label = f"{class_name}\n"
    
    # Добавляем атрибуты
    for attr in attributes:
        label += f"{attr}\n"
    
    # Добавляем первичный ключ
    label += f"PK: {primary_key}"

    dot.node(class_name, label=label, shape='record')

    # Сохраняем диаграмму в файл
    dot.render('er_diagram_test', format='png', cleanup=True)
    print("ER-диаграмма создана и сохранена как 'er_diagram.png'.")

# Пример использования
class_name = "User"
attributes = ["id", "name", "email", "created_at"]
primary_key = "id"

create_er_diagram(class_name, attributes, primary_key)

