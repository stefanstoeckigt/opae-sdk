#!/bin/bash -e
git fetch origin master
git merge --ff-only master
if [ $? == 0 ]; then
    echo "Successfully merged master"
    git push https://$GITHUB_SECRET_TOKEN@github.com/OPAE/opae-sdk.git coverity_scan > /dev/null 2>&1
else
    echo "Problem merging master"
    exit -1
fi



