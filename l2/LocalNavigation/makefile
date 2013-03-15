# Create OhmmNavigation java executable 
JFLAGS = -d bin -cp bin:jars/OHMM-newest.jar
JC = javac
.SUFFIXES: .java .class

SOURCES = $(wildcard src/*.java)
CLASSES = $(subst src/,bin/,$(SOURCES:.java=.class))

bin/%.class: src/%.java
	$(JC) $(JFLAGS) $<


all: $(CLASSES)
clean:
	rm bin/*.class
run:
	cd bin && java OhmmNavigation && cd ..
#DEPENDENCIES
bin/OhmmNavigation.class: bin/ServerRunner.class bin/OhmmMapServer.class
bin/ServerRunner.class: bin/OhmmMapServer.class
