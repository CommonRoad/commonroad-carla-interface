# Gitlab CI

## 1. Static Analysis
### Goals:
Check the code using as many code inspection methods as possible to keep the code in an optimal state.

### 1.1 Prospector:
One of the powerful static analysis tools for analyzing Python code and displaying information about errors, potential 
issues, convention violations and complexity. It includes:
- PyLint — Code quality/Error detection/Duplicate code detection
- pep8.py — PEP8 code quality
- pep257.py — PEP27 Comment quality
- pyflakes — Error detection
- mccabe — Cyclomatic Complexity Analyser
- dodgy — secrets leak detection
- pyroma — setup.py validator
- vulture — unused code detection
### Why:
It also contains many static check packages, which are very powerful, but are more difficult to pass and not conducive 
to step-by-step analysis. However, it can be set up through the `.prospector.yml` file.

#### 1.1.1 Flake8:
`Flake8` is a Python library that wraps `PyFlakes`, `pycodestyle` and `Ned Batchelder’s McCabe script`. 
It is a great toolkit for checking your code base against coding style (PEP8), programming 
errors (like “library imported but unused” and “Undefined name”) and to check cyclomatic complexity.
#### Why:
A very common and easy to use static check package containing three check libraries.
#### 1.1.2 pycodestyle (formerly pep8):
`pycodestyle` (formerly pep8) is a tool to check your Python code against some of the style conventions in PEP 8.
- Plugin architecture: Adding new checks is easy. 
- Parseable output: Jump to error location in your editor. 
- Small: Just one Python file, requires only stdlib. You can use just the pycodestyle.py file for this purpose. 
- Comes with a comprehensive test suite.
#### Why:
`pycodestyle` is not necessary, I add it for double-check.
#### 1.1.3 Pylint:
`Pylint` is a source-code, bug and quality checker for the Python programming language. It follows the style recommended 
by PEP 8, the Python style guide. It is similar to `Pychecker` and `Pyflakes`, but includes the following features:
- Checking the length of each line
- Checking that variable names are well-formed according to the project's coding standard
- Checking that declared interfaces are truly implemented.
#### Why:
Includes a code scoring system that is very intuitive.
#### 1.1.4 MyPy:
`Mypy` is an optional static type checker for Python that aims to combine the benefits of dynamic (or "duck") typing and 
static typing. `Mypy` combines the expressive power and convenience of Python with a powerful type system and compile-time 
type checking. `Mypy` type checks standard Python programs; run them using any Python VM with basically no runtime overhead.

`Mypy` type checks programs that have type annotations conforming to `PEP 484`.
The aim is to support almost all Python language constructs in `mypy`.
#### Why:
`Mypy` is an optional static type checker and it combines the benefits of dynamic (or "duck") typing and static typing. 
It is different from the code checker described above.

## 2. Dynamic Analysis
### 2.1 Unittest:
The goal of unit testing is to isolate each part of the program and show that the individual parts are correct.[1] A unit test provides a strict, written contract that the piece of code must satisfy. As a result, it affords several benefits.
#### test fixture:
A test fixture represents the preparation needed to perform one or more tests, and any associated cleanup actions. This may involve, for example, creating temporary or proxy databases, directories, or starting a server process.

#### test case:
A test case is the individual unit of testing. It checks for a specific response to a particular set of inputs. unittest provides a base class, TestCase, which may be used to create new test cases.

#### test suite:
A test suite is a collection of test cases, test suites, or both. It is used to aggregate tests that should be executed together.

#### test runner:
A test runner is a component which orchestrates the execution of tests and provides the outcome to the user. The runner may use a graphical interface, a textual interface, or return a special value to indicate the results of executing the tests.