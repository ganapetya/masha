import os
import sqlite3
from PyQt5.QtSql import QSqlDatabase, QSqlQuery

servo_num = 18  # 原代码支持6个舵机，现在调整为18
new_servos = {
    19: 500,
    20: 720,
    21: 130,
    22: 150,
    23: 500,
    24: 500
}

old_path = '/home/ubuntu/software/actionset_editor/ActionGroups'
new_path = old_path + '/new'

def servo_id_change(act_old, act_new):
    conn = sqlite3.connect(act_new)
    c = conn.cursor()
    
    # 构建新的表结构，增加 19-24 号舵机
    columns = ', '.join([f"Servo{i} INT" for i in range(1, 25)])
    c.execute(f"""
        CREATE TABLE ActionGroup(
            [Index] INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
            Time INT,
            {columns}
        );
    """)

    rbt = QSqlDatabase.addDatabase("QSQLITE")
    rbt.setDatabaseName(act_old)
    if rbt.open():
        actgrp = QSqlQuery()
        if actgrp.exec("SELECT * FROM ActionGroup"):  # 查询原始数据
            while actgrp.next():
                action_data = [str(actgrp.value(i)) for i in range(1, servo_num + 2)]
                
                # 添加 19-24 号舵机的默认值
                for i in range(19, 25):
                    action_data.append(str(new_servos[i]))
                
                insert_sql = f"INSERT INTO ActionGroup(Time, {', '.join([f'Servo{i}' for i in range(1, 25)])}) VALUES(" + ', '.join(action_data) + ");"
                c.execute(insert_sql)
            
            conn.commit()
            conn.close()
    rbt.close()

if __name__ == '__main__':
    d6a_list = [f for f in os.listdir(old_path) if f.endswith('.d6a')]
    if not os.path.exists(new_path):
        os.makedirs(new_path)
    
    for f in d6a_list:
        old = os.path.join(old_path, f)
        new = os.path.join(new_path, f)
        servo_id_change(old, new)
        print(f'{old} ----> {new}')
