# PUSH_TO_GITHUB_GUIDE.md

## Complete Guide: Pushing to GitHub

This guide explains how to push all the project files to the `https://github.com/Angel97977/Applying-Optimal-Control-In-Mujoco` repository.

---

## Table of Contents

1. [Initial Setup](#initial-setup)
2. [File Organization](#file-organization)
3. [Standard Push Workflow](#standard-push-workflow)
4. [Troubleshooting](#troubleshooting)

---

## Initial Setup

### Prerequisites

1. **GitHub Account**: https://github.com/Angel97977/Applying-Optimal-Control-In-Mujoco
2. **Git Installed**: https://git-scm.com/download
3. **SSH Keys or Personal Access Token** (for authentication)

### Verify Git Installation

```bash
git --version  # Should show v2.0+
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Setup SSH (Recommended) or HTTPS

**Option 1: SSH (Recommended)**

```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t ed25519 -C "your.email@example.com"

# Add to GitHub
cat ~/.ssh/id_ed25519.pub
# Copy to GitHub Settings → SSH Keys → Add New SSH Key

# Test connection
ssh -T git@github.com  # Should say "successfully authenticated"
```

**Option 2: HTTPS (Simpler)**

```bash
# GitHub will prompt for credentials (use personal access token)
# Create token at: Settings → Developer settings → Personal access tokens
```

---

## File Organization

### Current Directory Structure (pushing_to_angel/)

```
pushing_to_angel/
├── README.md                      # Main project documentation
├── GETTING_STARTED.md             # Installation & quick start
├── MATHEMATICAL_FRAMEWORK.md      # Detailed mathematical derivations
├── CONTROLLER_GUIDE.md            # Implementation guide
├── API_REFERENCE.md               # (will create)
├── requirements.txt               # Python dependencies
└── PUSH_TO_GITHUB_GUIDE.md        # This file
```

### GitHub Repository Structure (Target)

The repository should contain:

```
Applying-Optimal-Control-In-Mujoco/
├── README.md
├── GETTING_STARTED.md
├── MATHEMATICAL_FRAMEWORK.md
├── CONTROLLER_GUIDE.md
├── API_REFERENCE.md
├── requirements.txt
├── LICENSE                        # (already in repository)
│
├── src/                           # Core source code (from reference repo)
│   ├── __init__.py
│   ├── dynamics.py
│   ├── estimator_ekf.py
│   ├── controller_pmp.py
│   ├── controller_lqg.py
│   ├── controller_mpc.py
│   ├── simulator.py
│   └── trajectory_generator.py
│
├── examples/                      # Executable examples
│   ├── run_mujoco.py
│   ├── run_web.py
│   └── meta.py
│
├── tests/                         # Test suite
│   └── test_all.py
│
├── docs/                          # Additional documentation
│   ├── ARCHITECTURE.md
│   ├── GAIT_CONTROL.md
│   ├── TROUBLESHOOTING.md
│   └── PERFORMANCE_ANALYSIS.md
│
└── web/                           # Web interface files
    └── (HTML/CSS files)
```

---

## Standard Push Workflow

### Step 1: Clone the Repository Locally

```bash
# Clone using SSH (recommended)
git clone git@github.com:Angel97977/Applying-Optimal-Control-In-Mujoco.git
cd Applying-Optimal-Control-In-Mujoco

# Or using HTTPS:
# git clone https://github.com/Angel97977/Applying-Optimal-Control-In-Mujoco.git
```

### Step 2: Copy Documentation Files

Copy the documentation files from `pushing_to_angel/` to the root directory:

```bash
# From the repository root:
cp /path/to/pushing_to_angel/README.md .
cp /path/to/pushing_to_angel/GETTING_STARTED.md .
cp /path/to/pushing_to_angel/MATHEMATICAL_FRAMEWORK.md .
cp /path/to/pushing_to_angel/CONTROLLER_GUIDE.md .
cp /path/to/pushing_to_angel/requirements.txt .

# Verify
ls -la *.md
ls -la requirements.txt
```

### Step 3: Copy Source Code Files

If you have the source code (from the reference repository), copy it:

```bash
# Copy source modules
cp -r /path/to/source/src/* src/

# Copy examples
cp -r /path/to/source/examples/* examples/

# Copy tests (if available)
cp -r /path/to/source/tests/* tests/

# Copy web files (if available)
cp -r /path/to/source/web/* web/

# Copy docs (if available)
cp -r /path/to/source/docs/* docs/
```

### Step 4: Create .gitignore (Recommended)

```bash
cat > .gitignore << 'EOF'
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
venv/
*.egg-info/
dist/
build/

# IDE
.vscode/
.idea/
*.sublime-project
*.sublime-workspace

# OS
.DS_Store
.DS_Store?
Thumbs.db

# Results and logs
results/
logs/
*.png
*.csv
*.log

# Jupyter
.ipynb_checkpoints/
*.ipynb

# MuJoCo
.mujoco/
EOF
```

### Step 5: Check Status and Stage Files

```bash
# See what's new/modified
git status

# Stage all documentation and source files
git add README.md GETTING_STARTED.md MATHEMATICAL_FRAMEWORK.md CONTROLLER_GUIDE.md requirements.txt
git add src/
git add examples/
git add tests/
git add docs/
git add web/
git add .gitignore

# Verify staging
git status
# (Should show all new files as "Changes to be committed")
```

### Step 6: Create Initial Commit

```bash
git commit -m "Complete Quadruped Optimal Control implementation with comprehensive documentation

- Add three controller implementations: PMP, LQG, MPC
- Comprehensive README with usage instructions
- Getting started guide with troubleshooting
- Mathematical framework documentation
- Controller implementation guide
- Requirements and dependencies
- Source code with main runner (examples/run_mujoco.py)
- Support for multiple robots (Mini Cheetah, Go2, Aliengo)
- Automatic result visualization and metrics"
```

### Step 7: Push to GitHub

```bash
# Push to remote (GitHub)
git push -u origin main

# Verify (should see "100% (X) done" message)
```

### Step 8: Verify on GitHub

Visit: https://github.com/Angel97977/Applying-Optimal-Control-In-Mujoco

- ✓ Files visible on web interface
- ✓ README.md displayed on main page
- ✓ All directories present
- ✓ License file present

---

## Workflow for Updates

### Making Changes and Pushing Updates

```bash
# Edit a file (e.g., fix in README.md)
nano README.md

# Check what changed
git status
git diff README.md

# Stage the change
git add README.md

# Commit with descriptive message
git commit -m "Fix typo in installation instructions"

# Push
git push origin main
```

### Adding New Files

```bash
# Create new file
echo "New documentation" > docs/NEW_FEATURE.md

# Add and commit
git add docs/NEW_FEATURE.md
git commit -m "Add documentation for new feature"

# Push
git push origin main
```

---

## Branches (Optional)

### Create Feature Branches

```bash
# Create and switch to new branch
git checkout -b feature/add-custom-controller

# Make changes
# ... edit files ...

# Commit on branch
git add src/controller_custom.py
git commit -m "Add custom controller implementation"

# Push branch
git push origin feature/add-custom-controller

# (On GitHub, create Pull Request to merge into main)
# After review:
git checkout main
git pull origin main
git merge feature/add-custom-controller
git push origin main
```

---

## Troubleshooting

### Issue 1: "Permission denied (publickey)"

**Problem:** SSH key not set up correctly

**Solution:**
```bash
# Test SSH connection
ssh -T git@github.com

# If fails, add SSH key to GitHub
# Settings → SSH and GPG keys → New SSH Key
# Copy from: cat ~/.ssh/id_ed25519.pub
```

### Issue 2: "fatal: not a git repository"

**Problem:** Not in repository directory

**Solution:**
```bash
cd Applying-Optimal-Control-In-Mujoco
git status  # Should now work
```

### Issue 3: "Updates were rejected"

**Problem:** Remote has changes not in local

**Solution:**
```bash
# Fetch latest from remote
git fetch origin

# Merge remote into local
git merge origin/main

# Fix any conflicts
# Then push
git push origin main
```

### Issue 4: "filename too long" (Windows)

**Problem:** Windows path length limit

**Solution:**
```bash
# Enable long paths
git config --global core.longpaths true
```

---

## Quick Reference for Regular Pushes

**One-liner for simple changes:**
```bash
git add -A && git commit -m "Your message" && git push origin main
```

**Check before pushing:**
```bash
git status
git log --oneline -5
```

---

## Repository Settings (Optional)

### Enable GitHub Pages (Optional)

If you want documentation hosted on GitHub Pages:

1. Go to Settings → Pages
2. Select "main" branch / "root" folder
3. Wait for build to complete
4. Visit: https://angel97977.github.io/Applying-Optimal-Control-In-Mujoco/

### Add Topics (Optional)

In repository settings, add topics like:
- `optimal-control`
- `quadruped`
- `mujoco`
- `lqg`
- `mpc`
- `control-theory`

### Add Description

In about section: "Optimal control framework for quadruped stabilization in MuJoCo with PMP, LQG, and MPC implementations"

---

## Advanced: GitHub Actions (Optional)

```bash
# Create workflow directory
mkdir -p .github/workflows

# Create test action
cat > .github/workflows/python-tests.yml << 'EOF'
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2
    - uses: actions/setup-python@v2
      with:
        python-version: '3.10'
    - run: pip install -r requirements.txt
    - run: python -m pytest tests/
EOF

# Commit
git add .github/workflows/
git commit -m "Add GitHub Actions CI"
git push origin main
```

---

## Final Checklist

Before considering the push complete:

- [ ] README.md displays correctly on GitHub
- [ ] All .md files are present and readable
- [ ] requirements.txt is in root
- [ ] src/ directory with all modules
- [ ] examples/ directory with run_mujoco.py
- [ ] tests/ directory (if available)
- [ ] docs/ directory with additional guides
- [ ] .gitignore prevents committing unwanted files
- [ ] LICENSE file present
- [ ] No large binary files committed (keep repo < 100 MB)
- [ ] Latest commit message is descriptive

---

## Success!

Once your push is complete, your repository is live and ready for:
- ✅ Sharing with collaborators
- ✅ Documentation in GitHub Pages (optional)
- ✅ Issue tracking and discussions
- ✅ Version control and history
- ✅ Collaboration via pull requests

---

## Next Steps After Push

1. **Create Issues** for enhancement requests or bugs
2. **Add Collaborators** (Settings → Collaborators)
3. **Create Releases** for stable versions (Releases → Create Release)
4. **Write Discussions** for community engagement
5. **Update README** with results and comparison plots

---

For detailed information about the project, see [README.md](README.md)
