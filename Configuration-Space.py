import matplotlib.pyplot as plt 
import numpy as np   
import matplotlib 
from shapely.geometry import Polygon

W1 = 1;   #half width of link 1
L1 = 0.1; #half thickness of link 1

W2 = 0.5; #half width of link 2
L2 = 0.1; #half thickness of link 2

obstacle1 = Polygon([(1,1), (1.5,1), (1.5,1.5), (1, 1.5)])
obstacle2 = Polygon([(-1,0), (-3,0), (-3,-0.8), (-1, -0.8), (0.1, -0.4)])
# obstacle2 = Polygon([(-2,-0.5), (-2.5,-2), (-2.5,-2.5), (-2, -2.5)])


# Making Polygon as a obstacle
obs1 = [(1,1), (1.5,1), (1.5,1.5), (1, 1.5)]
obs1.append(obs1[0])   #repeating the first point to create a 'closed loop'
xs1, ys1 = zip(*obs1)  #creating lists of x and y values

obs2 = [(-1,0), (-3,0), (-3,-0.8), (-1, -0.8), (0.1, -0.4)]
# obs2 = [(-2,-0.5), (-2.5,-2), (-2.5,-2.5), (-2, -2.5)]
obs2.append(obs2[0])  #repeating the first point to create a 'closed loop'
xs2, ys2 = zip(*obs2) #creating lists of x and y values

theta1 = 30
theta2 = 20
i = (theta1*(np.pi))/180   #converting theta1 degree to radians for theta1
j = (theta2*(np.pi))/180   #converting theta2 degree to radians for theta1
# Finding corresponding four vertices of the each link_1 and link_2
# Link_1 has four vertices pt1, pt2 ,pt3 and pt4 
pt1_x = -L1*np.sin(i) 
pt1_y = L1*np.cos(i)
pt4_x = L1*np.sin(i)
pt4_y = -L1*np.cos(i)
pt2_x = (2*W1*np.cos(i)) - L1*np.sin(i)
pt2_y = (2*W1*np.sin(i)) + L1*np.cos(i)
pt3_x = (2*W1*np.cos(i)) + L1*np.sin(i)
pt3_y = (2*W1*np.sin(i)) - L1*np.cos(i)

# Link_2 has four vertices pt5, pt6 ,pt7 and pt8 
pt5_x = -L2*np.sin(i+j) + (2*W1*np.cos(i)) 
pt5_y =  L2*np.cos(i+j) + (2*W1*np.sin(i)) 
pt8_x =  L2*np.sin(i+j) + (2*W1*np.cos(i))
pt8_y = -L2*np.cos(i+j) + (2*W1*np.sin(i))
pt6_x = (2*W2*np.cos(i+j)) - L2*np.sin(i+j) + (2*W1*np.cos(i))
pt6_y = (2*W2*np.sin(i+j)) + L2*np.cos(i+j) + (2*W1*np.sin(i))
pt7_x = (2*W2*np.cos(i+j)) + L2*np.sin(i+j) + (2*W1*np.cos(i))
pt7_y = (2*W2*np.sin(i+j)) - L2*np.cos(i+j) + (2*W1*np.sin(i))


# for first Link (Link_1) storing the vertices in a 2D array
first_link = [(pt1_x , pt1_y), (pt2_x, pt2_y), (pt3_x, pt3_y), (pt4_x, pt4_y)]
first_link.append(first_link[0]) 
xs3, ys3 = zip(*first_link) 

# for second Link (Link_2) storing the vertices in a 2D array
second_link = [(pt5_x , pt5_y), (pt6_x, pt6_y), (pt7_x, pt7_y), (pt8_x, pt8_y)]
second_link.append(second_link[0]) 
xs4, ys4 = zip(*second_link) 

# Drawing the plot of robot world to display the robot and some polygonal obstacle_1 and obstacle_2
plt.figure()
plt.grid()
plt.fill(xs1,ys1, 'r') 
plt.fill(xs2,ys2, 'b')
plt.fill(xs3,ys3, 'k')
plt.fill(xs4,ys4, 'k') 
plt.xlabel('X -(distnace)') 
plt.ylabel('Y -(distnace)') 
plt.title('2 R Manipulator(in black) and Robot World with obstacles(in red)') 
plt.show() 


# Part2 of Assignment finding the Robot End Effector WorkSpace Code
# An array named theta1 storing the angles from 0 to 2PI in increment of 0.1 radians 
theta1 = np.arange(0, 2*(np.pi), 0.1)
theta2 = np.arange(0, 2*(np.pi), 0.1) #similarly for theta2


l1 = 2*W1
l2 = 2*W2
# An array storing all the values of coodinate corresponding to theta1 and theta2  
X = []
Y = []
for i in theta1:
    for j in theta2:
	    X.append(l1 * np.cos(i) + l2 * np.cos(i + j))  
	    Y.append(l1 * np.sin(i) + l2 * np.sin(i + j))  

   
plt.plot(X, Y) 
# naming the x axis 
plt.xlabel('X -(distnace)') 
# naming the y axis 
plt.ylabel('Y -(distnace)') 
# giving a title to my graph 
plt.title('Robot End Effector Workspace') 
plt.grid()
plt.show()



# An array named theta1 storing the angles from 0 to 2PI in increment of 0.06 radians 
theta1 = np.arange(0, 2*(np.pi), 0.06)
theta2 = np.arange(0, 2*(np.pi), 0.06)


THETA1 = [] # array containing the values of theta1 where collision occurs with the obstacle of link 
THETA2 = [] # array containing the values of theta2 where collision occurs with the obstacle of link

for i in theta1:
    for j in theta2:
    	# find the corresponding coordinate of vertices of link_1 and Link_2 in ground frame
	    pt1_x = -L1*np.sin(i) 
	    pt1_y = L1*np.cos(i)
	    pt4_x = L1*np.sin(i)
	    pt4_y = -L1*np.cos(i)
	    pt2_x = (2*W1*np.cos(i)) - L1*np.sin(i)
	    pt2_y = (2*W1*np.sin(i)) + L1*np.cos(i)
	    pt3_x = (2*W1*np.cos(i)) + L1*np.sin(i)
	    pt3_y = (2*W1*np.sin(i)) - L1*np.cos(i)

	    pt5_x = -L2*np.sin(i+j) + (2*W1*np.cos(i)) 
	    pt5_y =  L2*np.cos(i+j) + (2*W1*np.sin(i)) 
	    pt8_x =  L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt8_y = -L2*np.cos(i+j) + (2*W1*np.sin(i))	    
	    pt6_x = (2*W2*np.cos(i+j)) - L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt6_y = (2*W2*np.sin(i+j)) + L2*np.cos(i+j) + (2*W1*np.sin(i))
	    pt7_x = (2*W2*np.cos(i+j)) + L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt7_y = (2*W2*np.sin(i+j)) - L2*np.cos(i+j) + (2*W1*np.sin(i))

	    rod1 = Polygon([(pt1_x, pt1_y), (pt2_x, pt2_y), (pt3_x, pt3_y), (pt4_x, pt4_y)])
	    rod2 = Polygon([(pt5_x, pt5_y), (pt6_x, pt6_y), (pt7_x, pt7_y), (pt8_x, pt8_y)])
	    if rod1.intersects(obstacle1)  or rod2.intersects(obstacle1) :
	        THETA1.append(i*(180/(np.pi)))
	        THETA2.append(j*(180/(np.pi)))  
        

plt.plot(THETA1, THETA2, 'ro') 

THETA1 = [] # array containing the values of theta1 where collision occurs with the obstacle of link 
THETA2 = [] # array containing the values of theta2 where collision occurs with the obstacle of link

for i in theta1:
    for j in theta2:
    	# find the corresponding coordinate of vertices of link_1 and Link_2 in ground frame
	    pt1_x = -L1*np.sin(i) 
	    pt1_y = L1*np.cos(i)
	    pt4_x = L1*np.sin(i)
	    pt4_y = -L1*np.cos(i)
	    pt2_x = (2*W1*np.cos(i)) - L1*np.sin(i)
	    pt2_y = (2*W1*np.sin(i)) + L1*np.cos(i)
	    pt3_x = (2*W1*np.cos(i)) + L1*np.sin(i)
	    pt3_y = (2*W1*np.sin(i)) - L1*np.cos(i)

	    pt5_x = -L2*np.sin(i+j) + (2*W1*np.cos(i)) 
	    pt5_y =  L2*np.cos(i+j) + (2*W1*np.sin(i)) 
	    pt8_x =  L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt8_y = -L2*np.cos(i+j) + (2*W1*np.sin(i))	    
	    pt6_x = (2*W2*np.cos(i+j)) - L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt6_y = (2*W2*np.sin(i+j)) + L2*np.cos(i+j) + (2*W1*np.sin(i))
	    pt7_x = (2*W2*np.cos(i+j)) + L2*np.sin(i+j) + (2*W1*np.cos(i))
	    pt7_y = (2*W2*np.sin(i+j)) - L2*np.cos(i+j) + (2*W1*np.sin(i))

	    rod1 = Polygon([(pt1_x, pt1_y), (pt2_x, pt2_y), (pt3_x, pt3_y), (pt4_x, pt4_y)])
	    rod2 = Polygon([(pt5_x, pt5_y), (pt6_x, pt6_y), (pt7_x, pt7_y), (pt8_x, pt8_y)])
	    if rod1.intersects(obstacle2)  or rod2.intersects(obstacle2) :
	        THETA1.append(i*(180/(np.pi)))
	        THETA2.append(j*(180/(np.pi)))  
        

plt.plot(THETA1, THETA2, 'bo') 
plt.grid()
plt.xlabel('\u03B81  (in degrees)') 
plt.ylabel('\u03B82  (in degrees)')  
plt.title('Configuration Space ') 

plt.show()


   


