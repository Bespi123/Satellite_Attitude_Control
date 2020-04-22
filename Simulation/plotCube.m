function plotCube(roll, pitch, yaw)
 
 vertex_matrix = [0 0 0
                  1 0 0
1 1 0
0 1 0
0 0 1
1 0 1
1 1 1
0 1 1]-0.5;
 
faces_matrix = [1 2 6 5
2 3 7 6
3 4 8 7
4 1 5 8
1 2 3 4
5 6 7 8];
 
subplot(1,3,1)
axis([-1 1 -1 1 -1 1]);
axis equal off;
cube = patch('Vertices',vertex_matrix,'Faces',faces_matrix,'FaceColor', 'green');
rotate(cube,[1,0,0], roll);
rotate(cube,[0,1,0], pitch);
rotate(cube,[0,0,1], yaw);
view(0,0);
 
end