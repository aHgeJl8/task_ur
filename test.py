from graphviz import Digraph

def create_er_diagram(class_name, attributes, primary_key):
    # Создаем новый граф
    dot = Digraph()

    # Форматируем метку с классом и атрибутами
    label = f"{class_name} | {{"
    
    # Добавляем атрибуты
    for attr in attributes:
        if attr == primary_key:
            label += f"<f0> {attr} (PK) | "
        else:
            label += f"<f0> {attr} | "
    
    label = label.rstrip(" | ") + "}}"

    # Добавляем класс как узел
    dot.node(class_name, label=label, shape='record')

    # Сохраняем диаграмму в файл
    dot.render('er_diagram', format='png', cleanup=True)
    print("ER-диаграмма создана и сохранена как 'er_diagram.png'.")

# Пример использования
class_name = "User"
attributes = ["id", "name", "email", "created_at"]
primary_key = "id"

create_er_diagram(class_name, attributes, primary_key)
