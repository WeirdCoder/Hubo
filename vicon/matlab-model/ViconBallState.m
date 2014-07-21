classdef ViconBallState < LCMCoordinateFrame
  properties (GetAccess = public, SetAccess = private)
    x
    t
    aggregator
    defaultChannel
  end
  methods
    function obj = ViconBallState(r)
      num_u = getNumStates(r);
      dim = num_u;
      obj = obj@LCMCoordinateFrame('ViconBall', dim, 'x');
      lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      lc.subscribe('ViconBall', obj.aggregator);
      obj.setLCMCoder(ViconBallStateEncoder(obj.getCoordinateNames()));
      obj.x = [0,0,0,0,0,0,0,0,0,0,0,0];
      obj.t = 0;
    end
    function chan = defaultChan(obj)
      chan = obj.defaultChannel;
    end
    function [x,t] = getNextMessage(obj,timeout)
      data = obj.aggregator.getNextMessage(timeout);
      [x,t] = obj.lcmcoder.decode(data);
      if t ~= 0
            obj.x = x;
            obj.t = t;
      else
          x = obj.x;
          t = obj.t;
      end
      
    end
    function [x,t] = getCurrentValue(obj)
      x = obj.x;
      t = obj.t;
    end
  end
end
