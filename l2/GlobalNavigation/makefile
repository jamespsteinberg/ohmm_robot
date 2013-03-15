# Create GlobalNavigation java executable 
JFLAGS = -d bin -cp bin:bin/l2:jars/OHMM-newest.jar
JC = javac
.SUFFIXES: .java .class

SOURCES = $(wildcard src/*.java)
CLASSES = $(subst src/,bin/,$(SOURCES:.java=.class))

bin/%.class: src/%.java
	$(JC) $(JFLAGS) $<


all: $(CLASSES)
clean:
	rm bin/l2/*.class
run:
	cd bin && java l2/GNav && cd ..
