file_finder = find . -type f $(1) -not \( -path '*/venv/*' -o -path './external/*' \)

CMAKE_FILES = $(call file_finder,-name "*.cmake" -o -name "CMakeLists.txt")
PY_FILES = $(call file_finder,-name "*.py")
CPP_FILES = $(call file_finder,-regex '.*\.\(cpp\|hpp\|cu\|c\|h\)')

check: check_format pylint

format:
	$(PY_FILES) | xargs black
	$(CMAKE_FILES) | xargs cmake-format -i
	$(CPP_FILES) | xargs clang-format --style=file -i

check_format:
	$(PY_FILES) | xargs black --diff --check
	$(CMAKE_FILES) | xargs cmake-format --check

pylint:
	$(PY_FILES) | xargs pylint --rcfile=.pylintrc

.PHONY: test
test:
	python3 -m unittest discover -s script/evaluation
