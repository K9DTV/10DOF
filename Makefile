.PHONY: test ci

PYTHON ?= python3

test:
	$(PYTHON) -m pytest -q

ci: test
