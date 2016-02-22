%Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use.

%This is a script that initializes a blank sequence message, adding some 
%paths and performing some IPC functions. Use this at the top of all of 
%your scripts.
init_seqmsg

%This loop initializes the relevant fields for each quadrotor you are
%flying. 
for c=1:numquads
    
    clear seq %clear out previous sequence sent to low level matlab
    seq(1) = seq_default(); %intialize all fields to blank or default

    %just sit and idle
    seq(end).type = 901; %this is the TYPE that determines what happens in the low level code
    seq(end).time = t_inf;%this tells the quad to hover until another message is received
    
    %PUT OTHER FIELDS HERE
    
    
    
    %END OTHER FIELDS HERE
    
    seqM(c).seq = seq; %the variable seqM is a struct with metadata about the sequence
    seq_cntM(c) = 1;
end

%This packs the message and sends it via IPC. Do not change this.
ipcm.type = 4;
ipcm.qn = 1:numquads;
ipcm.seq = seqM;
ipcm.seqcnt = ones(numquads,1);

sipcm = serialize(ipcm);

ipcAPIPublish(msg_name,sipcm);



