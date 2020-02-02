# BMX Bike Robot Team Repo


## Get you Computer set up:

1. Follow these [instructions](https://github.com/dlynch7/Tiva_Make#tiva_make) to get all of the stuff you need to build the source code. This is a separate directory from the actually project folder.

2. Create a new directory on you computer to hold the project code. I called mine `bmx_bike`

3. Clone this repository to the directory you just created.

  ```
  cd ~/bmx_bike # or whatever you named it
  git clone https://github.com/rencheckyoself/bmx-bike.git
  ```

  If you haven't, send me your username so I can add you as a collaborator and give you edit permissions.

4. Create a development branch for yourself called `devel_XXX` where XXX are your initials. This is the branch where you will do all development and only push to `master` branch when the code is functional and tested.

  ```
  git branch devel_XXX
  ```

  Use `git checkout <branch-name>` to switch between branches and use `git merge <branch-name>` to merge changes from `<branch-name>` to the branch you have checked out.

  [If you have never used git before here is a lot of good info and some examples to help](http://robotics.mech.northwestern.edu/~elwin/git_intro.html)


5. Edit the makefile to match the locations for everything installed in step 1.

 - I have added the Makefile to the .gitignore list as not not overwrite for people developing on different OS. If you make any required changes to that please inform everyone.


6. Grab one of the dev boards we have and test if you've set things up properly. You should be able to complete everything on Dan's page up unitl the `make flash` and `make screen` commands at the very bottom.



## Development Workflow

This is an outline for the general process we should all follow when starting to develop a new feature:

First, make sure your local master branch is up to date with the remote master branch on github.com
```
git checkout master
git fetch origin master
git merge
```

Then merge any changes on to your development branch:

```
git checkout devel_XXX
git merge master
```

Now your development branch has the most up to date functioning code and you can start developing. If someone pushes an update to the master branch while you are doing your development, follow the same process above and resolve any conflicts.

Be sure to commit often during your development but only push your development branch stuff
```
git add <any new files or directories go here>
git commit -a # type a commit message at the top of your text editor
git push origin devel_XXX
```

Once you have finished and verified your development is functional, merge changes back to your local master branch and push to the online repository. **Before making a commit to master, ensure you don't have any unmerged changes and reach out to team members so we don't overwrite any progress.** It's fixable, just a headache :)

```
git checkout master
git merge devel_XXX

# Do a final check to make sure you will not be overwriting stuff on master

git push origin master
```
