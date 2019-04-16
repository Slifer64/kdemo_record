function Quat = makeQuatContinuous(Quat)

n_data = size(Quat,2);

Q_prev = Quat(:,1);
for j=2:n_data
   a =  Quat(:,j)'*Q_prev;
   
   if (a<0), Quat(:,j) = -Quat(:,j); end
   Quat(:,j) = Quat(:,j) / norm(Quat(:,j));
   
   Q_prev = Quat(:,j);
end


end