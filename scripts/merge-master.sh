#!/bin/bash -e

git merge --ff-only origin/master
if [ $? == 0 ]; then
    echo "Successfully merged master"
    git push https://$GITHUB_SECRET_TOKEN@github.com/OPAE/opae-sdk.git > /dev/null 2>&1
else
    echo "Problem merging master"
    exit -1
fi



