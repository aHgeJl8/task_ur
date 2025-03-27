from graphviz import Digraph

def create_er_diagram():
    # Создаем новый граф
    dot = Digraph(format='svg')

    # Добавляем сущность "Users"
    with dot.subgraph(name='cluster_users') as c:
        c.attr(style='filled', color='lightgrey')
        c.node_attr.update(style='filled', color='white')
        c.node('user_id', label=r'PK\nid')
        c.node('user_name', label=r'name')
        c.node('user_role_id', label=r'role_id')

    # Добавляем сущность "Roles"
    with dot.subgraph(name='cluster_roles') as c:
        c.attr(style='filled', color='lightgrey')
        c.node_attr.update(style='filled', color='white')
        c.node('role_id', label=r'PK\nid')
        c.node('role_name', label=r'name')

    # Добавляем связь между сущностями
    dot.edge('user_role_id', 'role_id', label='1:N')

    # Сохраняем диаграмму в файл
    dot.render('er_diagram', cleanup=True)
    print("ER-диаграмма создана и сохранена как 'er_diagram.svg'.")

# Создаем ER-диаграмму
create_er_diagram()
