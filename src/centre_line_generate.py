import numpy as np
start="<node name=\"$(eval 'centre_line_"
mid_1="')\" pkg=\"gazebo_ros\" type=\"spawn_model\" respawn=\"false\" output=\"screen\"\nargs=\"-sdf -file $(find taxinet)/models/line_dashed/model.sdf -model centre_line_"
mid_2=" -x "
end=" -y 0.0 -z 0.0 -Y 1.57\"/>\n"

runway_length=100/2.0
line_length=0.25
gap_length=0.15

start_pos=0.0#-(runway_length/2)
pos=start_pos

f=open('centreline.txt','w')

for i in np.arange(int(runway_length/(line_length+gap_length))):
   x_pos=pos+line_length/2.0
   x_pos_str=str(x_pos)
   string=start+str(i)+mid_1+str(i)+mid_2+x_pos_str+end
   f.write(string)
   pos+=(line_length+gap_length)