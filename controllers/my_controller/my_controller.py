from controller import Robot
import pandas as p
import random
import math
import operator
import matplotlib.pyplot as plt
import time

bool =True
class KNN:
            #################################
            def ReadDataSet(self , filename):
                d= p.read_csv(filename, sep=',')
                return d

            ####################################################
            def distanceEuclidienne(self,line1 , line2 , length):
                distance=0         
                for x in range(length):
                    if ((type(line1[x]) == str) | (type(line2[x]) == str)):
                        if (line1[x]==line2[x]):
                            distance=0
                        else:
                            distance=1
                    else:
                        distance += pow((line1[x]-line2[x]),2)
                                
                return math.sqrt(distance)

            ###################################################
            def VoisinKNN(self,instanceTest , DataSet , k):
                distance = []
                # dans le length de test on mis -1 psk le test ne contient pas de classe(label)
                length = len(instanceTest)-1
                for x in range(len(DataSet)):
                    dist = KNN.distanceEuclidienne(self,instanceTest,DataSet[x],length)
                    distance.append((DataSet[x],dist,x))
                distance.sort(key=operator.itemgetter(1))
                    
                voisins=[]
                for x in range (k+1):
                    voisins.append(distance[x][0])
                return voisins

            ##################################################
            def ClassifyI (self,Dataset , instance,K):
                voisins=KNN.VoisinKNN(self,instance , Dataset , K).copy()
                VoisinOccurance = {}
                for y  in range (len(voisins)):
                    #voisins[x][-1] le x correspond a la ligne et le -1 on travaille par modulo il correspond a la derniere case(c'est la classe)
                    ClasseChoisie = voisins[y][-1]
                    if ClasseChoisie in VoisinOccurance:
                        VoisinOccurance[ClasseChoisie]+=1
                    else:
                        VoisinOccurance[ClasseChoisie]=1

                VoisinOccSorted = sorted(VoisinOccurance.items(),key=operator.itemgetter(1),reverse=True)
                for x in range(len(instance)):
                    if(x == (len(instance)-1)):
                        instance[x]=VoisinOccSorted[0][0]
                            
                return VoisinOccSorted[0][0]

            ######################################
            def ADDToList( self, train, instance):
                train.append(instance)
                return train
            
            #########################################
            def ADDToFile(self ,filename , instance):
            
                with open(filename,'a') as f:    
                    for i in range(0,len(instance)-1):
                        f.write(str(instance[i])+",")
                    f.write(str(instance[len(instance)-1]))    
                    f.write("\n")
                    f.close()
###############################################             
def graph_data(distance,step):
    
    #enregistrer le taux d'erreur
    if distance<0.55:
        erreur.append(0.55-distance)
    elif distance>0.9:
        erreur.append(distance-0.9)
    else :
        erreur.append(0)
        
    distances.append(distance)   
    steps.append(step)
############################################### 
def graph_making(distances,steps):

    plt.plot(steps,distances)
    plt.xlabel('steps')
    plt.ylabel('Distance')
    plt.title('The evolution of the distance between the robot and the wall')
    plt.show()         
###############################################
#Initialisation of the data_graph:
distances = []
steps = []
erreur = []

def run_robot(robot,bool):

    timestep = int(robot.getBasicTimeStep())
    max_speed = 8
    nb_step=0

    # enable motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    left_sensor = robot.getDevice("left")
    left_sensor.enable(timestep)

    middle_sensor = robot.getDevice("middle")
    middle_sensor.enable(timestep)
    
    #cree un objet
    Obj1=   KNN()

    #Appel d'une methode de la classe
    d = KNN.ReadDataSet(Obj1,r'dataset.arff')
    #values transform it to numpy array then tolist to a list
    dataset = d.values.tolist()
    
    while robot.step(timestep) != -1:
        
        # Enter here functions to read sensor data, like: 
        left_val = (left_sensor.getValue())/1000
        middle_val = (middle_sensor.getValue())/1000
        nb_step += 1
        
        left_speed = max_speed
        right_speed = max_speed
        t=time.time()
        
        instance=[middle_val,left_val,'']
        action = KNN.ClassifyI( Obj1, dataset , instance, 1 )
        print("Le temps de classification: "+format(time.time()-t))
                
        
        print("capteur milieu : "+format(middle_val)+"      capteur gauche : "+format(left_val))
        print("action prédite : "+format(action))
        print("**********************************************************"*3)
        
        KNN.ADDToFile(Obj1,r'newDataKNN.txt', instance)
        graph_data(left_val,nb_step)
        
        if action=="Move-Forward":
            left_speed = max_speed
            right_speed = max_speed
            left_motor.setVelocity(-left_speed)
            right_motor.setVelocity(-right_speed) 
            bool=True
        else :
            if action=="Sharp-Right-Turn" :  
                left_speed = max_speed
                right_speed = max_speed/8 
                left_motor.setVelocity(-left_speed)
                right_motor.setVelocity(-right_speed)
                bool =False
            else :
                if action=="Slight-Right-Turn" :  
                    left_speed = max_speed
                    right_speed = max_speed/4
                    left_motor.setVelocity(-left_speed)
                    right_motor.setVelocity(-right_speed)   
                else :
                    if action=="Slight-Left-Turn" :
                        if bool == True:
                         left_speed = max_speed/2
                         right_speed = max_speed
                         left_motor.setVelocity(-left_speed)
                         right_motor.setVelocity(-right_speed)                          
        pass
            
        if nb_step == 5000 :
            
            somme_accuracy = 0
            for i in erreur:
                somme_accuracy += i
            print("--------------------------------------------------------"*3)    
            print("accuracy : " + str(1 - (somme_accuracy)/len(erreur)))
            print("--------------------------------------------------------"*3)
            graph_making(distances,steps) 
            graph_making(erreur,steps)     

        
if __name__ == "__main__":

    my_robot = Robot()
    run_robot(my_robot,bool)