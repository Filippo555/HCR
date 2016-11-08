#!/bin/bash  
git add * 
read -p "Commit description: " desc  
git commit --all -m "$desc"  
git push

 
