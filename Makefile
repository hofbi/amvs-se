.PHONY: test
test:
	python3 -m unittest discover -s script/evaluation
	python3 -m unittest discover -s script/cqp

format:
	shfmt -l -w .

check:
	shfmt -d .
