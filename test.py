from graphviz import Digraph

def create_er_diagram(classes):
    # Создаем новый граф
    dot = Digraph(format='png')

    # Добавляем классы
    for class_name, attributes in classes.items():
        # Формируем метку для узла
        label = f"{class_name} | {{"
        for attr in attributes:
            if attr == attributes[0]:
                label += f"<f0> {attr} (PK) | "
            else:
                label += f"<f0> {attr} | "
        label = label.rstrip(" | ") + "}}"

        # Добавляем класс как узел
        dot.node(class_name, label=label, shape='record')

    # Добавляем связи
    dot.edge('User', 'Role', constraint='true')

    # Сохраняем диаграмму в файл
    dot.render('er_diagram', cleanup=True)
    print("ER-диаграмма создана и сохранена как 'er_diagram.png'.")

# Пример использования
classes = {
    'User': ['id', 'name', 'role_id'],
    'Role': ['id', 'name']
}

create_er_diagram(classes)
