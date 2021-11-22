#!/bin/bash

pip install -r docs_requirements.txt
make html
xdg-open build/html/index.html
