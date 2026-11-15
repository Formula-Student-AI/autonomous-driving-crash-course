# Git Version Control

This is a basic introduction to Git, covering everything you need to start contributing code safely. It only covers a fraction of Git's functionality. For more, see [MIT's The Missing Semester – Version Control](https://missing.csail.mit.edu/2026/version-control/).

## 1. Basic Terminal Commands

The terminal is where you type commands to interact with your computer.

- **Directory** = a folder on your computer.
- **Current directory** = the folder you are currently working inside.
- `cd` = change directory (move into another folder).
- `ls` = list files and folders in your current directory.
- `mkdir` = create a new directory (folder).

```bash
cd Documents       # Move into Documents
ls                 # See what's inside
cd MyProject       # Move into MyProject
cd ..              # Go back one folder
```

## 2. Cloning a Repository

Git is a version control system that tracks changes to code and allows multiple people to collaborate.

- `git` tells the terminal to execute a Git command.
- `git clone` downloads a repository from GitHub.
- `git status` shows your current branch and any changes to your files.

Copy the repository URL from GitHub.

```bash
cd Documents
git clone <repository-url>
cd <repository-name>
git status
```

## 3. Branching

We use three types of branches:

- `main` – Stable, working version of the project.
- `dev` – Where approved team features are integrated.
- `feature/name` – Your own branch for developing a feature.

**Never develop directly on `main` or `dev`.**

Get the latest version of `dev` and create your own branch:

```bash
git switch dev
git pull origin dev
git switch -c feature/my-feature
```

`origin` is the default name Git gives to the remote repository you cloned from (GitHub).

- `git switch` changes branches.
- `-c` creates a new branch.
- `git branch` lists your local branches.
- `git branch -a` lists local and remote branches.
- `git branch --show-current` tells you which branch you are on.

**Important:** Git may prevent you from switching branches if your uncommitted changes would be overwritten. Commit your work before switching, or learn how to use `git stash`.

## 4. Saving and Uploading Your Work

After making changes:

```bash
git status                       # See what changed
git diff                         # Inspect your changes
git add .                        # Stage changes for committing
git commit -m "Describe changes" # Save a checkpoint locally
git push -u origin feature/my-feature  # Upload to GitHub
```

After your first push, you can simply use `git push`.

**Add files you don't want Git to track to `.gitignore`** (e.g. `.venv/`, `__pycache__/`, API keys and large generated files).

## 5. Getting Other People's Changes

To update your feature branch with the latest team code:

```bash
git switch dev
git pull origin dev
git switch feature/my-feature
git merge dev
```

This downloads the latest `dev` from GitHub and merges it into your feature branch.

## 6. Pull Requests – Getting Code into dev or main

- Go to GitHub and open a Pull Request (PR).
- Set your feature branch as the source and `dev` as the destination.
- Request at least one teammate to review your changes.
- Test your code and address any feedback before merging.
- Only merge `dev` into `main` through an approved PR.
- Never push directly to `main` or `dev`.

## 7. Safety Rules

- Always check `git status` and your current branch before making changes.
- Commit and push regularly to avoid losing work and keep your remote branch updated.
- Never blindly accept merge conflicts.
- Be extremely careful with `git push --force` and `git reset --hard`: they can overwrite or destroy work. Avoid them unless you fully understand the consequences.

**Merge conflicts:** If a conflict occurs, Git cannot automatically combine changes. Open the affected files and identify the conflicting sections (marked by `<<<<<<<`, `=======` and `>>>>>>>`). Decide what the final code should be, remove the markers, test your changes, then run:

```bash
git add <resolved-file>
git commit -m "Resolve merge conflict"
```

**Please Note:** AI can help write and debug code, but learn Git independently. Never blindly copy Git commands from AI. Git protects the entire team's work, so you must understand how to use it yourself.