# Flatland



## Important Note: They released a new version of Flatland Library on 6th Sept, 2019. 

### Update by: 

  * In terminal, enter

  ``` conda activate flatland-rl ```

  then 

  ``` pip install -U flatland-rl ```

---

## Useful links: 

### - The challenge website: 
https://www.aicrowd.com/challenges/flatland-challenge

### - Start kit (Flatland Library Page):
http://flatland-rl-docs.s3-website.eu-central-1.amazonaws.com/readme.html

### - Useful Discussions:

* planning vs re-scheduling
https://discourse.aicrowd.com/t/discussion-planning-vs-re-scheduling/1647
* Additional Programming language
https://discourse.aicrowd.com/t/additional-programming-language/1684

---

## Round 1 Plan

### Generate instances and render solution
1. Run **round1.py**:
```bash
$ cd Round 1
$ python3 round1.py 
```
2. Run ECBS:
```bash
$ cd ECBS
$ ./ECBS -m ../[map_name.txt]  -a ../[agent_name.txt] -o ../[path_name.txt] -t 60 -w 1.2 --makespan [number]
```
* where num = (width + height) * 1.5
3. Run **temp_render_path.py**:
```bash
$ python3 temp_render_path.py --config [config_name.pkl] --map [map_name.txt] --agent [agent_name.txt] --path [path_name.txt]
```

### - To-Do:
1. Move the working code from round 1 notebook to the run.py for round 1
2. Write a multi-threading python code for running 4 solvers on 4 cpus. 
   1. All Terminate when once one of solvers finds the solution? May need to confirm.
3. Pack everything and try local evaluation
4. Try submission.





---

## Round 2 Plan

### - To-Do:

  * ...


