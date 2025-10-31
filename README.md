# Code for ME449 Assignment 3

- Code for this assignment is in `assignment_3.py`
- output CSV's are in `output/*`

## Written answers:
- Part 2 answers are written in the PDF below
- Explanation why convergence is difficult from the long iterates initial guess:
    - Convergence is difficult here because the angles in the initial guess are all individually very far from the
      correct angles which would lead to convergence. This means that Newton-Raphson must bounce around the FK function
      many times until it finally ends up in the well that leads to a zero of the function. 

## run instructions:
### using `uv` (no virtualenv needed)
```
uv sync
uv run assignment_3.py
```

### using `pip` (recommended: use virtualenv)
```
pip install -r requirements.txt
python assignment_3.py
```

Author: Conor Hayes
