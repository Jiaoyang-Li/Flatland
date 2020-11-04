import glob, os, math
import pandas as pd 
import random, sys
from typing import List



lns_folder = "./lns_data/"
run_csv = "./285.csv"
total_time_limit = 28800
no_tests = 36
no_lns_tests = 28
iteration_distribution = [0] * no_tests
time_limit = 580
max_iterations = 5000
increase_units = [10,20,50,100,200,500]
amplify_lns_improve = 1


#simulate annealing parameter

T = 1
Tmin = 0.0001
numIterations = 500
alpha = 0.999
change_neighbours = 5


class cell:
    iteration: int
    time: float
    cost: int #total normalized cost over all levels
    n_cost: float #total normalized cost over all levels
    improve:float #total improve on percentage over all levels
    no_instaces: int

    def __init__(self,iteration,time,cost,n_cost,improve):
        self.iteration = iteration
        self.time = time
        self.cost = cost
        self.n_cost  = n_cost
        self.improve = improve
        self.no_instaces = 1

    def avg_time(self):
        return  self.time/self.no_instaces
    def avg_improve(self):
        improve =  (self.improve/self.no_instaces)
        if (improve==1):
            return 1
        return  improve * amplify_lns_improve

#load lns data
def load_data(lns_folder):
    lns_files = glob.glob(lns_folder+"*.csv")
    print("Load from ",len(lns_files)," files")

    lns_data = [ [cell(i,0,0,0,1) for i in range(0,max_iterations)] for t in range(0,no_tests)  ]

    for lns in lns_files:
        filename = os.path.basename(lns).split(".")[0].split("_")

        test = int(filename[1])
        level = int(filename[2])


        f = open(lns)
        last_cost = 0
        last_improve = 0.0
        last_n_cost = 0.0
        last_time = 0
        index = 0
        inital_n_cost = 0
        initial_time = 0
        for line in f.readlines():
            if line.startswith("g"):
                continue
            row = [ float(x) for x in line.split(",") if x!="\n"]
            time = row[2]
            cost = row[5]
            n_cost = 1 - row[7]
            if index == 0:
                improve = 1
                inital_n_cost = n_cost
                initial_time = time
            else:
                improve = n_cost/inital_n_cost

            lns_data[test][index].time += time-initial_time
            lns_data[test][index].cost += cost
            lns_data[test][index].n_cost += n_cost
            lns_data[test][index].improve += improve
            lns_data[test][index].no_instaces += 1

            last_cost = cost
            last_improve = improve
            last_n_cost = n_cost
            last_time = time-initial_time
            index +=1


        for i in range(index+1,len(lns_data[test])):
                    lns_data[test][i].time += last_time
                    lns_data[test][i].cost += last_cost
                    lns_data[test][i].n_cost += last_n_cost

                    lns_data[test][i].improve += last_improve

                    lns_data[test][i].no_instaces += 1
        f.close()
    return lns_data

class run_cell:
    test: int
    level: int
    runtime: float
    reward: float


    def __init__(self,test,level,runtime,reward):
        self.test = test
        self.level = level
        self.runtime = runtime
        self.reward  = reward

def load_run(file):
    data = pd.read_csv(file)
    run_data = [[None for x in range(0,10)] for i in range(0,no_tests) ]
    sum_reward = 0
    sum_time = 0
    for index,row in data.iterrows():
        name = row["case"]
        runtime = row["runtime"]
        reward = row["reward"]

        name = name.split("/")
        test = int(name[0].split("_")[1])
        level = int(name[1].split("_")[1])
        run_data[test][level] = run_cell(test,level,runtime,reward)
        sum_reward+=reward
        sum_time +=runtime
    return run_data







def getReward(distribution,lns_data:List[List[cell]], run_data:List[List[run_cell]]):
    total_reward=0.0
    total_time = 0.0
    for test in range(0,len(run_data)):
        iteration = distribution[test]
        lns_time = lns_data[test][iteration].avg_time()
        lns_improve = lns_data[test][iteration].avg_improve()

        for level in range(0,len(run_data[test])):
            if total_time > total_time_limit or run_data[test][level]==None:
                return total_reward

            total_time += (run_data[test][level].runtime + lns_time)
            total_reward += (run_data[test][level].reward * lns_improve)
    return total_reward





class state:
    distribution: list
    total_reward: float

    def __init__(self, distribution,total_reward):
        self.distribution = distribution[:]
        self.total_reward = total_reward

    def get_neighbour(self):

        neighbours = change_neighbours
        new_distribution =  self.distribution[:]
        while neighbours>0:

            new_iteration = -1
            i = -1

            while new_iteration<0 or new_iteration >= max_iterations:
                i = random.randint(1,no_lns_tests-1)
                direction = random.choice([-1,1])
                increase_unit = random.choice(increase_units)
                new_iteration = new_distribution[i] + direction*increase_unit
            assert(i>=0)

            new_distribution[i] = new_iteration
            neighbours-=1
        neighbour = state(new_distribution,0)
        return neighbour




# start simulated annealing
def optimize_time(T,Tmin,alpha,numIterations,lns_data,run_data):

    current = state(iteration_distribution[:],0)
    print("calculate reward")
    current.total_reward = getReward(current.distribution,lns_data,run_data)
    max = current
    print("Start")
    print(current.distribution,current.total_reward)


    while (T > Tmin):
        for  i in range(0,numIterations):

            if current.total_reward > max.total_reward:
                max = current
                print("Better solution: ",max.distribution,max.total_reward,T)

            new: state = current.get_neighbour()
            new.total_reward = getReward(new.distribution,lns_data,run_data)

        delta = new.total_reward - current.total_reward

        ap = math.e **(delta/T)
        if ap >= random.random():
            current = new

        T = T* alpha

    print(max.distribution, max.total_reward)
    return max



# load data from file
run_data = load_run(run_csv)
all = 0

lns_data = load_data(lns_folder)


# for i in lns_data[5]:
#     print(i.time,i.avg_time(),i.improve,i.avg_improve())


optimize_time(T,Tmin,alpha,numIterations,lns_data,run_data)

    
    
    