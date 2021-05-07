clc; clear all; close all;

s = {};
exp1 = {};
exp1.Static = {177:191};
exp1.Dynamic = {1:176};

exp2 = {};
exp2.Static = {1:31};
exp2.Dynamic = {};

s.Experiments = {exp1 exp2};
encoded_json = jsonencode(s);

fid = fopen("metadata.json",'w');
fprintf(fid, encoded_json); 

%%

fileName = 'filename.json'; % filename in JSON extension
str = fileread(fileName); % dedicated for reading files as text
data = jsondecode(str); % Using the jsondecode function to parse JSON from string


