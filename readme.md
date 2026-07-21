# Patient Mobility Project A0509

Robot description and MoveIt config packages for Patient Mobility Project using [Doosan A0509](https://www.doosanrobotics.com/en/product-solutions/product/a-series/a0509/)


## Package Dependencies
- [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) (**branch**: `humble` **commit**: [`37cc855`](https://github.com/DoosanRobotics/doosan-robot2/tree/37cc855b8c860d0367bbe88c6b7e139817724225))
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) (**branch**: `master` **commit**: [`a66e227`](https://github.com/stereolabs/zed-ros2-wrapper/tree/a66e227e290fb26b9e21b0f45aedcbf93ace52b9))
- [zed-ros2-interfaces](https://github.com/stereolabs/zed-ros2-interfaces) (**branch**: `master` **commit**: [`efd12ae`](https://github.com/stereolabs/zed-ros2-interfaces/tree/efd12ae97dbb7e313c2e909698a0664396f0069a))


## Dev Setup
### Prerequisites
- [CMake](https://cmake.org/download/)
- [Python](https://www.python.org/downloads/) >= 3.10
- [pip](https://pypi.org/project/pip/)

### Instructions
#### 1. Clone
```bash
git clone https://github.com/wsu-pmp/pmp-a0509.git
```

#### 2. Install Dev Dependencies
*Optionally: Create a virtual environment for Python (e.g. [venv](https://docs.python.org/3/library/venv.html), [virtualenv](https://virtualenv.pypa.io/en/latest/))*

```bash
pip install -r requirements-dev.txt
```

#### 3. Install pre-commit Git Hooks Locally
```bash
pre-commit install
pre-commit install --hook-type commit-msg
```

### Python Formatting / Linting
Formatting, linting, import sorting with [Ruff](https://github.com/astral-sh/ruff).

#### Check Formatting / Linting
```
make check
```

#### Fix Formatting / Linting
```
make fix
```

### Commit Hooks
#### On Pre-Commit
- [no-commit-to-branch](https://github.com/pre-commit/pre-commit-hooks)
    - targeting `main`
- [check-yaml](https://github.com/pre-commit/pre-commit-hooks)
- [check-xml](https://github.com/pre-commit/pre-commit-hooks)
- [ruff-check](https://github.com/astral-sh/ruff-pre-commit)
- [ruff-format](https://github.com/astral-sh/ruff-pre-commit)


#### On Commit Message
- [commit-msg](https://github.com/jorisroovers/gitlint)
    - Ignore [`title-must-not-contain-word`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#t5-title-must-not-contain-word)
    - Ignore [`body-is-missing`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#b6-body-is-missing)
    - Ignore [`body-changed-file-mention`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#b7-body-changed-file-mention)
