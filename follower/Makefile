CXXFLAGS = -std=c++11 -O2 -g -Wall -fmessage-length=0 -I../

SRCS = follower.cpp ../follower_local.cpp ../DroneUtil.cpp

LDLIBS = -ldronecode_sdk -ldronecode_sdk_action -ldronecode_sdk_offboard -ldronecode_sdk_telemetry 

TARGET = follower

OBJS=$(subst .cpp,.o,$(SRCS))

all:	$(TARGET)

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	$(CXX) $(CXXFLAGS) -MM $^>>./.depend;

$(TARGET):	$(OBJS)
	$(CXX) $(LDFLAGS) -o $(TARGET) $(OBJS) $(LDLIBS)

clean:
	rm -f $(OBJS) $(TARGET) ./.depend

include .depend
