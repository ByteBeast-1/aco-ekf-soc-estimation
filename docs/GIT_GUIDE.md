# Git & GitHub — Complete Beginner Setup Guide

This walks you from "never used git" to "pushed my first commit," and
then gives you the ongoing weekly habit to follow for the rest of the
project.

---

## Step 1 — Install Git

**Windows**
1. Download from https://git-scm.com/download/win
2. Run the installer — default options are fine for everything.
3. Open "Git Bash" from the Start menu (this is the terminal you'll use).

**Mac**
1. Open Terminal and run:
   ```
   git --version
   ```
2. If it's not installed, macOS will prompt you to install Xcode Command
   Line Tools — accept that. (Or install via `brew install git` if you
   have Homebrew.)

**Linux**
```bash
sudo apt update
sudo apt install git
```

Check it worked, on any OS:
```bash
git --version
```

---

## Step 2 — Tell Git who you are (one-time setup)

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

Use the **same email** you'll use for your GitHub account — this is how
GitHub matches your commits to your profile later.

---

## Step 3 — Create a GitHub account

1. Go to https://github.com and sign up (if you don't already have an
   account).
2. Verify your email.

This is also the account recruiters will eventually look at — pick a
professional username if you haven't already.

---

## Step 4 — Create the repository on GitHub

1. Click the **+** icon (top right) → **New repository**.
2. Repository name: `aco-ekf-soc-estimation` (or whatever you prefer).
3. Description (optional but recommended): *"Multi-objective ACO-tuned
   EKF for lithium-ion battery SoC estimation."*
4. Choose **Public** (so recruiters can see it).
5. **Do NOT** tick "Add a README" — you already have one from this
   project skeleton, and it's simplest to avoid conflicts.
6. Click **Create repository**.
7. GitHub will show you a page with a URL like:
   ```
   https://github.com/your-username/aco-ekf-soc-estimation.git
   ```
   Keep this page open — you'll need that URL in Step 6.

---

## Step 5 — Unzip the project skeleton

Unzip the project folder you got from this conversation. You should see:
```
aco-ekf-soc-estimation/
├── README.md
├── .gitignore
├── LICENSE
├── models/
├── simulation/
├── ekf/
├── aco/
├── results/
└── docs/
```

Open a terminal **inside this folder**:
```bash
cd path/to/aco-ekf-soc-estimation
```

---

## Step 6 — Turn this folder into a git repo and push it

Run these commands one at a time, from inside the folder:

```bash
# 1. Initialize git in this folder
git init

# 2. Stage all files (tell git "I want to track these")
git add .

# 3. Make your first commit (a snapshot with a message)
git commit -m "Initial commit: project skeleton + Week 1 battery model"

# 4. Rename the default branch to main (modern convention)
git branch -M main

# 5. Connect this folder to the GitHub repo you created
git remote add origin https://github.com/your-username/aco-ekf-soc-estimation.git

# 6. Push your commit up to GitHub
git push -u origin main
```

Replace the URL in step 5 with **your actual repo URL** from Step 4.

Refresh your GitHub repo page in the browser — your files should now be
there. This is your first commit, done.

---

## Step 7 — Your ongoing weekly workflow

Every time you finish meaningful progress (ideally at least once per
week, matching your 5-week plan):

```bash
# See what changed
git status

# Stage the changed files
git add .

# Commit with a clear, specific message
git commit -m "Add baseline EKF predict-correct loop, validated vs Coulomb counting"

# Push to GitHub
git push
```

### Writing good commit messages

| Bad | Good |
|---|---|
| `update` | `Add MOACO weighted-sum pheromone update` |
| `fix` | `Fix SoC clamping bug in battery_params.m` |
| `week 3` | `Reproduce base paper's single-objective ACO (Q=2.7e-6, R=0.044)` |

Specific, present-tense, one line describing *what changed and why*.

---

## Step 8 — Tag your milestones (optional but impressive)

After each major week's work is solid:

```bash
git tag -a v0.1-baseline-ekf -m "Working baseline EKF"
git push origin v0.1-baseline-ekf
```

Suggested tags for your project:
- `v0.0-battery-model` (Week 1)
- `v0.1-baseline-ekf` (Week 2)
- `v0.2-aco-reproduced` (Week 3)
- `v0.3-moaco` (Week 4)
- `v1.0-results` (Week 5)

This lets anyone browsing your repo jump straight to "the version where
X was working" instead of digging through commit history.

---

## Step 9 — (Optional, once comfortable) Branches for experiments

If you want to try something risky without breaking your working code:

```bash
git checkout -b feature/moaco-pareto-dominance
# ... make changes, commit as normal ...
git push -u origin feature/moaco-pareto-dominance
```

If it works out, merge it back:
```bash
git checkout main
git merge feature/moaco-pareto-dominance
git push
```

If it doesn't work out, just delete the branch — `main` was never
touched, so nothing is lost.

---

## Cheat sheet — commands you'll actually use 95% of the time

```bash
git status          # what's changed?
git add .            # stage everything changed
git commit -m "..."   # save a snapshot with a message
git push              # send it to GitHub
git pull              # get latest changes (if working across 2 machines)
git log --oneline     # see your commit history
```

---

## Prefer a visual tool instead of the terminal?

**GitHub Desktop** (https://desktop.github.com) does everything above
through buttons instead of commands — good if you want to get moving
immediately and pick up the terminal commands later. Either is fine;
the terminal workflow above is worth learning eventually since it's
the version every recruiter and engineering team assumes you know.
