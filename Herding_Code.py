import numpy as np

import math

import matplotlib.pyplot as plt

import imageio



class flock():

    def flocking_python(self):

        N = 100 #No. of Boids

        frames = 300 #No. of frames

        limit = 100 #Axis Limits

        L  = limit*2

        P = 10 #Spread of initial position (gaussian)

        V = 10 #Spread of initial velocity (gaussian)

        delta = 1 #Time Step

        c1 = 0.0001 #Attraction Scaling factor

        c2 = 0.01 #Repulsion scaling factor

        c3 = 1 #Heading scaling factor

        c4 = .01 #Randomness scaling factor

        vlimit = 1 #Maximum velocity

        object_count = 1

        object_spread = 50

        object_repulsion = c2 * 20

        images = [] #for plotting a gif of movements

        boundary = True

        boundary_range = 5



        #Initialize

        p = P*np.random.randn(2,N)

        v = V*np.random.randn(2,N)

        object_p = object_spread*np.random.randn(2, object_count)
        object_v = np.random.randn(2, object_count)



        #Initializing plot

        plt.ion()

        fig = plt.figure()

        ax = fig.add_subplot(111)





        for i in range(0, frames):

            v1 = np.zeros((2,N))

            v2 = np.zeros((2,N))

            

            #YOUR CODE HERE

            #Calculate Average Velocity v3
            v3 = [np.sum(v[0, :]) * c3 / N, np.sum(v[1, :]) * c3 / N]


            if (np.linalg.norm(v3) > vlimit): #limit maximum velocity

                v3 = [v3[0] * (vlimit/np.linalg.norm(v3)), v3[1] * (vlimit/np.linalg.norm(v3))]



            for n in range(0, N):

                for m in range(0, N):

                    if m!=n:

                        #YOUR CODE HERE

                        #Compute vector r from one agent to the next
                        r = p[:, m] - p[:, n]


                        if r[0] > L/2:

                            r[0] = r[0]-L

                        elif r[0] < -L/2:

                            r[0] = r[0]+L



                        if r[1] > L/2:

                            r[1] = r[1]-L

                        elif r[1] < -L/2:

                            r[1] = r[1]+L



                        #YOUR CODE HERE

                        #Compute distance between agents rmag
                        rmag = math.sqrt(r[0]**2 + r[1]**2)

                        #Compute attraction v1
                        v1[:, n] = v1[:, n] + (c1 * r) / rmag

                        #Compute Repulsion [non-linear scaling] v2
                        v2[:, n] = v2[:, n] - (c2 * r / (rmag ** 2))

                
                for m in range(0, object_count):

                    #YOUR CODE HERE

                    #Compute vector r from one agent to the next
                    r = object_p[:, m] - p[:, n]


                    if r[0] > L/2:

                        r[0] = r[0]-L

                    elif r[0] < -L/2:

                        r[0] = r[0]+L



                    if r[1] > L/2:

                        r[1] = r[1]-L

                    elif r[1] < -L/2:

                        r[1] = r[1]+L



                    #YOUR CODE HERE

                    #Compute distance between agents rmag
                    rmag = math.sqrt(r[0]**2 + r[1]**2)

                    #Compute Repulsion [non-linear scaling] v2
                    v2[:, n] = v2[:, n] - (object_repulsion * r / (rmag ** 2))

                   

                #YOUR CODE HERE

                #Compute random velocity component v4
                v4 = c4 * np.random.randn(2)


                #v2[:, n] = v2[:, n] - (c2 * r / (rmag ** 2))
                if boundary:
                    rad = (p[0, n] - L/2 + boundary_range)
                    v2[0, n] = v2[0, n] - (c2 * 100 / (rad ** 2))
                    
                    rad = (p[0, n] + L/2 - boundary_range)
                    v2[0, n] = v2[0, n] + (c2 * 100 / (rad ** 2))
                    
                    rad = (p[1, n] - L/2 + boundary_range)
                    v2[1, n] = v2[1, n] - (c2 * 100 / (rad ** 2))
                    
                    rad = (p[1, n] + L/2 - boundary_range)
                    v2[1, n] = v2[1, n] + (c2 * 100 / (rad ** 2))


                

                #Update velocity
                v[:, n] = v1[:, n] + v2[:, n] + v3 + v4


                #Stop the agents at the wall
                if boundary:
                    if p[0, n] > (L/2 - boundary_range):
                        if v[0, n] > 0:
                            v[0, n] = 0
                    if p[0, n] < (-L/2 + boundary_range):
                        if v[0, n] < 0:
                            v[0, n] = 0
                    if p[1, n] > (L/2 - boundary_range):
                        if v[1, n] > 0:
                            v[1, n] = 0
                    if p[1, n] < (-L/2 + boundary_range):
                        if v[1, n] < 0:
                            v[1, n] = 0


            #Move predators
            center = [np.mean(p[0]), np.mean(p[1])]
            for n in range(object_count):
                r = center - object_p[:, n]
                rmag = math.sqrt(r[0]**2 + r[1]**2)

                if(rmag > 25):
                    object_v[:, n] = .01 * r
                else:
                    object_v[:, n] = 0
            
               

            #YOUR CODE HERE

            #Update position
            p = p + (delta * v)

            #object_p = object_p + (delta * object_v)


            #Periodic boundary

            tmp_p = p



            tmp_p[0, p[0,:]>L/2] = tmp_p[0,p[0,:]> (L/2)] - L

            tmp_p[1, p[1,:] > L/2] = tmp_p[1, p[1,:] > (L/2)] - L

            tmp_p[0, p[0,:] < -L/2]  = tmp_p[0, p[0,:] < (-L/2)] + L

            tmp_p[1, p[1,:] < -L/2]  = tmp_p[1, p[1,:] < (-L/2)] + L



            p = tmp_p

            # Can Also be written as:

            # p[p > limit] -= limit * 2

            # p[p < -limit] += limit * 2



            line1, = ax.plot(p[0, 0], p[1, 0])



            #update plot

            print(i)

            ax.clear()

            ax.quiver(p[0,:], p[1,:], v[0,:], v[1,:]) # For drawing velocity arrows

            if object_count > 0:

                ax.scatter(object_p[0, :], object_p[1, :], color = 'red') # For drawing objects

            plt.xlim(-limit, limit)

            plt.ylim(-limit, limit)

            line1.set_data(p[0,:], p[1,:])


            fig.canvas.draw()

            plt.show()

            plt.savefig('control' + str(i) + '.png')

            images.append(imageio.imread('control' + str(i) + '.png'))

        imageio.mimsave('/Users/JackHolland/Desktop/Boulder/CSCI4314/HW3/test.gif', images, fps = 25)



flock_py = flock()

flock_py.flocking_python()

