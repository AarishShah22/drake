classdef ContactForceTorqueSensor < TimeSteppingRigidBodySensor %& Visualizer
  
  properties
    normal_ind=[];
    tangent_ind=[];
    kinframe;
    manip;  % warning: could get stale, but keeping it around for the draw method
  end
  
  methods
    function obj = ContactForceTorqueSensor(tsmanip,body_ind,xyz,rpy)
      % default frame to initialize visualizer
%      if tsmanip.twoD
%        fr = CoordinateFrame('DefaultForceTorqueFrame',getNumStates(tsmanip)+3,'f');
%      else
%        fr = CoordinateFrame('DefaultForceTorqueFrame',getNumStates(tsmanip)+6,'f');
%      end
%      obj = obj@Visualizer(fr);
      obj.kinframe = RigidBodyFrame(body_ind,xyz,rpy);
      body = getBody(tsmanip,body_ind);
      obj.name = [body.linkname,'ForceTorque'];
    end
    
    function tf = isDirectFeedthrough(obj)
      tf = true;
    end
    
    function obj = compile(obj,tsmanip,manip)
      if (tsmanip.position_control) error('need to update this method for this case'); end

      body = getBody(manip,obj.kinframe.body_ind);
      
      if isempty(body.contact_pts)
        error('Drake:ContactForceTorqueSensor:NoContactPts','There are no contact points associated with body %s',body.linkname);
      end

%      obj = setInputFrame(obj,MultiCoordinateFrame({getStateFrame(tsmanip),constructFrame(obj,}));
      
      nL = sum([manip.joint_limit_min~=-inf;manip.joint_limit_max~=inf]); % number of joint limits
      nC = manip.num_contacts;
      nP = 2*manip.num_position_constraints;  % number of position constraints
      nV = manip.num_velocity_constraints;  

      num_body_contacts = size(body.contact_pts,2);
      contact_ind_offset = size([manip.body(1:obj.kinframe.body_ind-1).contact_pts],2);
      
      % z(nL+nP+(1:nC)) = cN
      obj.normal_ind = nL+nP+contact_ind_offset+(1:num_body_contacts);
      
      mC = 2*length(manip.surfaceTangents(manip.gravity)); % get number of tangent vectors

      % z(nL+nP+nC+(1:mC*nC)) = [beta_1;...;beta_mC]
      for i=1:mC
        obj.tangent_ind{i} = nL+nP+(mC*nC)+contact_ind_offset+(1:num_body_contacts);
      end
      
      obj.manip = manip;
    end
    
    function draw(obj,t,xft)
      if length(obj.xyz)~=2
        error('only implemented for the planar case so far');
      end
      
      xft = splitCoordinates(getInputFrame(obj),xft);
      x = xft{1}; ft = xft{2};

      kinsol = doKinematics(obj.manip,x(1:obj.manip.getNumDOF),false,false);
      
      body_pts = kinsol.T{obj.body}\[obj.xyz, obj.xyz+.001*ft(1:2); 1 1];  % convert force from sensor coords to body coords
      body_pts = body_pts(1:2,:);
      
      world_pts = forwardKin(obj.manip,kinsol,obj.body,body_pts);
      
      figure(1); clf; xlim([-2 2]); ylim([-1 3]); axis equal;
      axisAnnotation('arrow',world_pts(1,:),world_pts(2,:),'Color','r','LineWidth',2);
    end
    
    function y = output(obj,tsmanip,manip,t,x,u)
      z = tsmanip.solveLCP(t,x,u)/tsmanip.timestep;
      
      % todo: could do this more efficiently by only computing everything
      % below for indices where the normal forces are non-zero

      body = manip.body(obj.kinframe.body_ind);
      
      kinsol = doKinematics(manip,x(1:manip.getNumDOF));
      contact_pos = forwardKin(manip,kinsol,obj.kinframe.body_ind,body.contact_pts);
      
      [d,N] = size(contact_pos);
      [pos,~,normal] = collisionDetect(manip,contact_pos);

      % flip to sensor coordinates
      pos = frameKin(obj.kinframe,manip,kinsol,pos);
      sensor_pos = forwardKin(obj.kinframe,manip,kinsol,zeros(3,1));
      normal = frameKin(obj.kinframe,manip,kinsol,repmat(sensor_pos,1,N)+normal);
      tangent = manip.surfaceTangents(normal);

      % compute all individual contact forces in sensor coordinates
      force = repmat(z(obj.normal_ind)',d,1).*normal;
      mC=length(tangent);
      for i=1:mC
        force = force + repmat(z(obj.tangent_ind{i})',d,1).*tangent{i} ...
          - repmat(z(obj.tangent_ind{i+mC})',d,1).*tangent{i}; 
      end
      y = sum(force,2);

      if 0 %(tsmanip.twoD)
        torque = sum(cross([pos;zeros(1,N)],[force;zeros(1,N)]),2);
        y(3) = obj.jsign*torque(3);
      else
        y(4:6) = sum(cross(pos,force),2);
      end
    end
    
    function fr = constructFrame(obj,tsmanip)
      if 0 % tsmanip.twoD
        coords{1}=['force_',manip.x_axis_label];
        coords{2}=['force_',manip.y_axis_label];
        coords{3}='torque';
      else
        coords{1}='force_x';
        coords{2}='force_y';
        coords{3}='force_z';
        coords{4}='torque_x';
        coords{5}='torque_y';
        coords{6}='torque_z';
      end
      
      fr = CoordinateFrame(obj.name,6,'f',coords);
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.body = map_from_old_to_new(obj.body);
    end

  end
    
end