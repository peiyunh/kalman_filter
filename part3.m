for i = 0:9
  fprintf('Working on p3n%02d...\n', i); 
  doit_slam(sprintf('part3/p3n%02d', i));
end