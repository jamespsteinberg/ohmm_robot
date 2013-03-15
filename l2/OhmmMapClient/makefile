# Create Ohmm Map Client java executable
JFLAGS = -d bin -cp bin
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
	cd bin && java OhmmMapClient && cd ..
#DEPENDENCIES
bin/OhmmMapClient.class: bin/OhmmMap.class bin/MapPanel.class
bin/OhmmMap.class: bin/MapPanel.class
