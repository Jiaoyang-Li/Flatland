time=10
path="instances"

for i in $(seq 0 1 999) 
do 
	echo Instance $i
	./ECBS -m ${path}//map_${i}.txt  -a ${path}//agents_${i}.txt -o ${path}//path_${i}.txt -t ${time} -w 1.5 --makespan=300
done    
