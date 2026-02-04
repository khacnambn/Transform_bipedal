Makefile for bipedal robot project

.PHONY: help install install-dev test clean lint format

help:
	@echo "Bipedal Robot Project"
	@echo "======================"
	@echo "Available targets:"
	@echo "  install       - Install package"
	@echo "  install-dev   - Install with development dependencies"
	@echo "  test          - Run tests"
	@echo "  test-cov      - Run tests with coverage"
	@echo "  lint          - Run code linting"
	@echo "  format        - Format code with black"
	@echo "  clean         - Clean build files"
	@echo "  example-walk  - Run simple walking example"

install:
	pip install -e .

install-dev:
	pip install -e ".[dev]"

test:
	pytest tests/ -v

test-cov:
	pytest tests/ -v --cov=src/bipedal_robot --cov-report=html

lint:
	ruff check src/ tests/ examples/
	pylint src/bipedal_robot

format:
	black src/ tests/ examples/
	ruff check --fix src/ tests/ examples/

clean:
	rm -rf build/ dist/ *.egg-info
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

example-walk:
	python examples/simple_walk.py --steps 5 --speed 1.0

example-joint:
	python examples/joint_control.py
