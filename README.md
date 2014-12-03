The Renoir - Le robot de traçage
======

![Renoir Image](http://www.ibiblio.org/wm/paint/auth/renoir/renoir.jpg)


Tasks
------
* (Hojo) Hardware
  * Spring-loaded gripping thingy
* (DONE) Vectors to robot actions
  * Start with manually specified trajectories - Have baxter draw from a list of line segments
    * Have an algorithm to specify in which order lines are drawn in
    * Feed lines into MoveIt
  * Have a hard-coded picture to world transformation
* (DONE) Paper tracking
  * Calculate picture frame to world frame transformation. This can be a service that publishes a picture-to-world transformation matrix.
  * Corner detection (we can use 3/4 AR tags)
  * Kalman Filter
* Actual drawing
  * Idea 1 : Preload an alphabet, have baxter write
  * Idea 2 : Given a bitmap image, just trace the pixels
  * Idea 3 : Image Vectorizer?
   * Converts bitmap images to a list of line segments

How to Run (Currently)
======
ALL CODE IS ON THE baxter BRANCH
1) Set up pic2world transforms
*  rosrun pic2world fix_baxter_cam.sh
*  roslaunch pic2world baxter_single.sh

2) Setup movit
* roslaunch draw draw_rviz.launch

3) Run code (currently draws a traingle)
* rosrun draw run.py

Git Help
======

Setup
----------------
Downloading this repository
  * git clone https://github.com/justinjfu/renoir.git
  
[Optional] Skip password prompts
  * Set up SSH keys: https://help.github.com/articles/generating-ssh-keys/
  * git remote set-url origin git@github.com:justinjfu/renoir.git

Simple git usage
----------------
  * Update local code from remote
    * git pull origin master
      * Equivalent to:
      * git fetch + git merge origin/master
  * Do your work
  * Add 
    * git add -A
    * git add file_name
  * Commit
    * git commit -m "Your commit message"
    * If you made a mistake,
      * git commit --amend
  * Synchronize to remote
    * git push origin master

Branches
----------
  * Create a branch
    * git checkout -b branch_name
    * git branch branch_name
  * Moving between branches
    * git checkout branch_name
  * Pushing/pulling from a branch
    * git pull/push origin branch_name
  * Merging a branch into master
    * Step 1: git checkout master
    * Step 2: git merge branch_name
