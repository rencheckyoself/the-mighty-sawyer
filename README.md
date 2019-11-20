# the-mighty-sawyer

# Getting started [Development]
Fork the GitHub Classroom repository from [here](https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git).  This means that you are creating a clone of the entire Classroom repo and placing a copy in your own personal GitHub repo; this is called a fork.  You will be mostly working with the fork you have created during development. 

Once you have forked our official project repo to your personal repo, you can now clone your personal repo to your local machine.
```
git clone https://github.com/<YOUR_USERNAME>/the-mighty-sawyer.git
```

To be able to interact with the official GitHub Classroom project repo, we have to take an additional step -- ie. add a remote upstream.

1. Open Terminal.
2. You can see the current configured remote repo for your fork by
```
git remote -v
> origin  https://github.com/YOUR_USERNAME/the-mighty-sawyer.git (fetch)
> origin  https://github.com/YOUR_USERNAME/the-mighty-sawyer.git (push)
```

3. Now you can set up a new remote `upstream` repo that can be synced with your fork
```
git remote add upstream https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git
```

4. Check whether you did this successfully 
```
git remote -v
> origin	https://github.com/lee-jm/the-mighty-sawyer.git (fetch)
> origin	https://github.com/lee-jm/the-mighty-sawyer.git (push)
> upstream	https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git (fetch)
> upstream	https://github.com/ME495-EmbeddedSystems/the-mighty-sawyer.git (push)
```