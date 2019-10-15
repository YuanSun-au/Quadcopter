
mdl = 'open_loop';
open_system(mdl)

io(1)=linio('open_loop/Thrust converter',1,'input');
io(2) = linio('open_loop/OUT1',1,'output');
io(3) = linio('open_loop/OUT2',1,'output');
io(4) = linio('open_loop/OUT3',1,'output');

io(5) = linio('open_loop/OUT4',1,'output');
io(6) = linio('open_loop/OUT5',1,'output');
  
opspec = operspec(mdl);
linsys = linearize(mdl,io,opspec);