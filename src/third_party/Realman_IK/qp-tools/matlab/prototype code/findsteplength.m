function alpha = findsteplength(s,Delta_s)

    Test_s = s./Delta_s;
    Test_s(s./Delta_s>0) = -1000;
	[~,idx] = max(Test_s);
	alpha = -Test_s(idx);


end