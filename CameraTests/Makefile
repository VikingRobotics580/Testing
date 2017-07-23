AT=@
CXX=arm-frc-linux-gnueabi-g++
HOME=C:\Users\VikingRobotics580
INCLUDES=-I$(HOME)/wpilib/cpp/current/include -I./inc
CPPFLAGS=-g -Wall -W $(INCLUDES) -std=c++14 -fPIC
LFLAGS=-L$(HOME)/wpilib/cpp/current/lib -lwpi
TEAM=580
RMCOM=rm -rf
TARGET=roboRIO-$(TEAM)-FRC.local

SOURCES=$(wildcard src/*.cpp)
OBJECTS=$(patsubst src/%.cpp,src/%.o,$(SOURCES))

all: $(OBJECTS)
	test -d bin/ || mkdir -p bin/
	$(CXX) $(CPPFLAGS) $(OBJECTS) $(LFLAGS) -o bin/FRCUserProgram

clean:
	@for dir in src; do find $$dir -name \*.o -exec $(RMCOMMAND) {} \; ; done
	$(RMCOMMAND) bin/*
	$(RMCOMMAND) src/*.o

deploy:
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(TARGET) "rm /home/lvuser/FRCUserProgram || true"
	sftp -o StrictHostKeyChecking=no -oBatchMode=no -b sftpbatchfile lvuser$(AT)$(TARGET)
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(TARGET) ". /etc/profile.d/natinst-path.sh; chmod a+x /home/lvuser/FRCUserProgram; /usr/local/frc/bin/frcKillRobot.sh -t -r"

kill-robot:
	ssh lvuser$(AT)$(TARGET) "/usr/local/frc/bin/frcKillRobot.sh"
