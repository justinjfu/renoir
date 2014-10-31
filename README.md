The Renoir - Le robot de traçage
======

![Renoir Image](http://www.ibiblio.org/wm/paint/auth/renoir/renoir.jpg)


Tasks
------
* (Hojo) Hardware
  * Spring-loaded gripping thingy
* (Sara) Vectors to robot actions
  * Start with manually specified trajectories - Have baxter draw from a list of line segments
    * Have an algorithm to specify in which order lines are drawn in
    * Feed lines into MoveIt
  * Have a hard-coded picture to world transformation
* (Justin) Image Vectorizer 
  * Converts bitmap images to a list of line segments

* [Stretch] Paper tracking
  * Calculate picture frame to world frame transformation.
  * Corner detection (we can use 3/4 AR tags)
  * Kalman Filter
* [Stretch] Feedback
  * Have Baxter know which lines are one the paper  


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
