data = ['Mag (V/m)','x_rel (mm)','y_rel (mm)','x_field (V/m)','y_field (V/m)','z_field (V/m)','x (m)','y (m)','z (m)','rx (rad)','ry (rad)','rz (rad)']

with open('saved_data/test_data.txt', 'w') as f:
    f.write('\t'.join(data))
    f.write('\n')