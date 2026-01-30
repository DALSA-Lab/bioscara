# Building the Documentation locally

It is recommended to use a virtual environment to manage all dependencies.
All following commands are executed from the *~/bioscara/docs/* directory.
```bash
cd docs
```


## Create the Virtual Environment and Install Dependencies
These steps need to be executed only once at the first time.
Create the virtual python environment:
```bash
python -m venv .venv
```

Before installing the dependencies the environment needs to be activated:
```bash
source .venv/bin/activate
```

The we can install the required packages to build the documentation:
```bash
pip install -r requirements-docs.txt
```

Aditionally install the required latex tools:
<!-- TODO -->

## Making the Documentation

First make sure the virtual environment is activated:
```bash
source .venv/bin/activate
```

Then run doxygen from the *docs/doxygen/* directory to produce HTML, XML and LaTeX output:
```bash
cd doxygen
doxygen
```

Compile the LaTeX to a PDF:
```bash
cd doxygen/latex
make
```

Finally build the Sphinx documentation:
```bash
sphinx-build -b html . build/html
```

It can also be compiled to a PDF:
```bash
sphinx-build -M latexpdf . build/latex
```

