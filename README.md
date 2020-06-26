# Flatland 2

---

## How to use for CBS solver development

The run.py in Flatland2020 folder is the python file for local testing. 

It has two ways to exchange information with C++ solver:

1. via boost API. (hasn't been implemented yet) 

   I left several arrays containing the map & agent information that is required by the C++ solver.

   (Line 168 - Line 294 of run.py)

2. via local .txt files

   They are saved in config folder. There is a boolean flag in the run.py triggering the saving function. 

   If the boost API hasn't been used, you can use the local .txt files for testing algorithms

For now, I am not sure how C++ solver sends the solution (paths) to the python code. By default, if you save the paths of agents in a "paths.txt" file in the Flatland2020/config folder. The run.py will read it automatically and convert them to actions. 

In the beginning of the run.py, there is a list of parameters that can change map structure, the number of agents etc. 

For visualization, you can change Line 403 for a different wait time for visualizing each timestep. or uncomment the code on Line 399 to hit the enter for next step visualization.  

---

## Known Flatland Library Bugs/Issues

1. Local test environment does not work if the evaluation process was interrupted.

Reason: redis-server recorded something about the evaluation data in the server cache. If the evalution was interrupted, the cache remains and causes issues when the user tries to rerun the evalution. 

Solution:**redis-cli flushall** to empty cache and rerun the evaluation. Then it works.

2. **Old Flatland Stochastic Data API does not work. Seems that they have changed the API.**

---

## Useful links: 

  ### - The challenge website: 
  https://flatland.aicrowd.com/intro.html 

  ### - Flatland Library Page (2.1.10, latest lib is 2.2.x):
   http://flatland-rl-docs.s3-website.eu-central-1.amazonaws.com/index.html 

  ### - Useful Discussions:

---

**Timeline:** 

- **June 1st - June 30th:** Warm-Up Round
- **July 1st - July 31st:** Round 1
- **August 1st - October 19th:** Round 2
- **October 20th - October 25th:** Post Challenge Analysis
- **October 25th:** Final Results Announced
- **October 16th - November 10th:** Post Challenge Wrap-Up

---

## Warm-Up Round Plan

  ### 6/23 -6/24  pick up the old solution and clean up the code

**6/24 - ...  ECBS, K-robust CBS, STN**

----

## Current Solution Structure

https://app.lucidchart.com/invitations/accept/156887d6-7e9f-43ca-af22

![image](Structure.png)

---

## Round 1 Plan

  ### ...


