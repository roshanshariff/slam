BEGIN{
FS=":"
  }
{
  split($1,s," ");
  map=s[3];
  getline; no=$2;
  split($1,s1," ");
  type=s1[1];
  getline; le=$2;
  getline; se=$2;
  getline; t=$2+0;
  index1=map" "type" "no;
  if (stat[index1]==0)
  {
    stat[index1]=1;
  }
  statistics(index1, "le", le);
  statistics(index1, "se", se);
  statistics(index1, "t", t);
}
END{
  for (i in stat)
    {
      print i":", "landmark error:", mean[i"#le"], stdev[i"#le"],
	"state error:", mean[i"#se"], stdev[i"#se"], 
	"time:", mean[i"#t"],stdev[i"#t"],
	"samples:", n[i"#se"];
    }
}

function statistics(index1, id, val)
{
  ind=index1"#"id;
  x[ind]+=val;
  xx[ind]+=val*val;
  n[ind]++;
  mean[ind]=x[ind]/n[ind];
  stdev[ind]=sqrt(xx[ind]/n[ind]-mean[ind]*mean[ind])
} 
