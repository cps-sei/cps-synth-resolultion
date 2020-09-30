CXXFLAGS = -std=c++11 -O2 -g -Wall -fmessage-length=0

SRCS = missionapp.cpp Enforcer.cpp ElasticEnforcer.cpp SigFun.cpp Signal.cpp TTIFun.cpp StlExpr.cpp ElasticStlEnforcer.cpp Coordinator.cpp DroneUtil.cpp SimpleCoordinator.cpp StateStore.cpp EnemyDrone.cpp StlEnforcer.cpp RunawayEnforcer.cpp BoundaryEnforcer.cpp DTTFun.cpp IntersectingCoordinator.cpp WeightedCoordinator.cpp RobustnessCoordinator.cpp DTGFun.cpp FlightEnforcer.cpp follower_local.cpp flyeightmission.cpp reconmission.cpp mission.cpp ReconEnforcer.cpp MissileEnforcer.cpp ReconFun.cpp PriorityCoordinator.cpp ConjunctionCoordinator.cpp json/jsoncpp.cpp

LDLIBS = -ldronecode_sdk -ldronecode_sdk_action -ldronecode_sdk_offboard -ldronecode_sdk_telemetry

TARGET = missionapp

OBJS=$(subst .cpp,.o,$(SRCS))
#RANDOM_OBJS=$(shell gshuf -e -- $(OBJS))

ifdef ZSRMMT_ROOT_DIR
LDFLAGS+=-L$(ZSRMMT_ROOT_DIR)
LDLIBS+=-lzsv
CXXFLAGS+=-DUSE_ZSRM=1 -I$(ZSRMMT_ROOT_DIR) -I./json/json
endif

all:	$(TARGET) follower

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	$(CXX) $(CXXFLAGS) -MM $^>>./.depend;

$(TARGET):	$(OBJS)
	$(CXX) $(LDFLAGS) -o $(TARGET) $(OBJS) $(LDLIBS)

follower: ./follower/*
	cd ./follower; make; cd ../

clean:
	rm -f $(OBJS) $(TARGET) ./.depend

include .depend
