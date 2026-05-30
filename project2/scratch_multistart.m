function scratch_multistart
cd('C:\Users\Administrator\Desktop\ROBOTFIN\ROBOT\project2');
params=robot_params('SR3');
q_list=deg2rad([30 -40 50 20 45 10;60 30 -50 -30 60 90;-45 60 40 60 -50 -60;120 -90 90 100 30 120;-90 45 -45 -90 60 45]);
base=deg2rad([0 0 0 0 0 0; 0 -45 45 0 45 0; 0 45 -45 0 -45 0; 90 0 0 0 60 0; -90 0 0 0 -60 0; 45 -45 45 45 45 45; -45 45 -45 -45 -45 -45]);
rng(7); extra=zeros(30,6); qlim=params.qlim; for i=1:30, extra(i,:)=qlim(:,1)'+rand(1,6).*(qlim(:,2)'-qlim(:,1)'); end
seeds=[base; extra];
opts=struct('max_iter',100,'tol',1e-8,'lambda',0.05,'verbose',false);
for i=1:size(q_list,1)
 T=forward_kinematics(q_list(i,:),params); best=inf; found=0;
 for s=1:size(seeds,1)
  try
   [q,info]=inverse_kinematics_numerical(T,seeds(s,:),params,opts);
   err=norm(wrap(q-q_list(i,:)));
   if err<best, best=err; end
   if info.converged && err<1e-3, found=1; break; end
  catch
  end
 end
 fprintf('%d best %.3g found %d\n',i,best,found);
end
end
function w=wrap(x), w=mod(x+pi,2*pi)-pi; end
scratch_multistart
