#include <iostream>
#include <fstream>
#include "./robot_movement.h"
#include <climits>
#include <vector>
#include <queue>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

struct pathNode{

	Face_handle triang;
	Face_handle  previous;
	pathNode(){
		triang=NULL;
		previous=NULL;
	}
};

void print_path(list<pathNode>* pathList,Face_handle f){

	
	CDT::Face_handle cur=f;
	CDT::Face_handle prev=NULL;
	list<pathNode>::iterator it;
	while(true){
		
		int i=1;
		for(it=pathList->begin();it!=pathList->end();++it){
			if(it->triang == cur){
				i=0;
				break;
			}
		}
		prev=it->triang;
		cur=it->previous;
		if(i==1)
			break;
		cout << (prev->vertex(0)->point()) << ";" 
				<< (prev->vertex(1)->point()) << ";" 
				<<  (prev->vertex(2)->point()) << endl;
	}

}

bool in_list(list<CDT::Face_handle> V,CDT::Face_handle fh){

	for(list<CDT::Face_handle>::iterator it=V.begin();
		it!=V.end();++it){
		if( fh->vertex(0)->point() ==  (*it)->vertex(0)->point()
		&& fh->vertex(1)->point() ==  (*it)->vertex(1)->point()
		&& fh->vertex(2)->point() ==  (*it)->vertex(2)->point()){
			return true;
		}
	}
	return false;

}

bool find_paths(CDT& cdt,CDT::Face_handle fh,Point& query,list<pathNode>* pathList){
	
	list<CDT::Face_handle> Q;
	list<CDT::Face_handle> V;
	
	pathNode* newNode=new pathNode();
	newNode->triang=fh;
	newNode->previous=NULL;
	pathList->push_back(*newNode);
	
	V.push_back(fh);
	Q.push_back(fh);
	while(Q.size() >0){
	
		CDT::Face_handle t=Q.front();
		Q.pop_front();
		

		
		Point2 points[3];
		points[0]=t->vertex(0)->point(); 
		points[1]=t->vertex(1)->point();
		points[2]=t->vertex(2)->point();
		if(check_inside(query,points,points+3, K())){
			cout << "The optimal path was found "<< endl;
			cout << "Printing the coordinates of the triangles that form it " << endl;
			//cout << (*(t->vertex(0))) << ";" << (*(t->vertex(1))) << ";" << (*(t->vertex(2))) << endl; 
			print_path(pathList,t);
			return true;
		}
		for(int i=0;i<3;i++)
			if(t->neighbor(i)->info().in_domain()){
				CDT::Face_handle u=t->neighbor(i);
				if(!in_list(V,u)){
					V.push_back(u);
					Q.push_back(u);
					
					pathNode* newNode1=new pathNode();
					newNode1->triang=u;
					newNode1->previous=t;
					pathList->push_back(*newNode1);
					//path->push_back(u);
				}
			}	
	}
	return false;

}


void 
mark_domains(CDT& ct, CDT::Face_handle start, int index, list<CDT::Edge>& border )
{
	if(start->info().nesting_level != -1){
		return;
	}
	list<CDT::Face_handle> queue;
	queue.push_back(start);
	while(! queue.empty()){
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if(fh->info().nesting_level == -1){
			fh->info().nesting_level = index;
			for(int i = 0; i < 3; i++){
				CDT::Edge e(fh,i);
				CDT::Face_handle n = fh->neighbor(i);
				if(n->info().nesting_level == -1)
					if(ct.is_constrained(e)){ 
						border.push_back(e);
//						fh->neighbor(i)->info().is_constr = true;
					}
					else 
						queue.push_back(n);
			}
		}
	}
}


void
mark_domains(CDT& cdt)
{
  for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
    it->info().nesting_level = -1;
    it->info().is_constr = false;
    it->info().visited=false;
  }
  list<CDT::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), 0, border);
  while(! border.empty()){
    CDT::Edge e = border.front();
    border.pop_front();
    CDT::Face_handle n = e.first->neighbor(e.second);
    if(n->info().nesting_level == -1){
      mark_domains(cdt, n, e.first->info().nesting_level+1, border);
    }
  }
}



void insert_polygon(CDT& cdt,const Polygon_2& polygon){
  if ( polygon.is_empty() ) return;
	Polygon_2::Vertex_iterator vit2=polygon.vertices_end();
	vit2--;
  CDT::Vertex_handle v_prev=cdt.insert(*vit2);
  for (Polygon_2::Vertex_iterator vit=polygon.vertices_begin();
       vit!=polygon.vertices_end();++vit)
  {
    CDT::Vertex_handle vh=cdt.insert(*vit);
    cdt.insert_constraint(vh,v_prev);
    v_prev=vh;
  }  
}

int check_inside(Point2 pt, Point2 *pgn_begin, Point2 *pgn_end, K traits)
{
  switch(CGAL::bounded_side_2(pgn_begin, pgn_end,pt, traits)) {
    case CGAL::ON_BOUNDED_SIDE :
		return 1;
    case CGAL::ON_BOUNDARY:
		return 1;
    case CGAL::ON_UNBOUNDED_SIDE:
		return 0;
  }
}	

bool locate_in_neighbourhood(CDT& cdt,Face_handle fh,Point& query,int traversed){
	
	int i=0;
	if(fh==NULL)
		return false;
	fh->info().visited=true;
	Point2 points[3];
	points[0]=fh->vertex(0)->point();
	points[1]=fh->vertex(1)->point();
	points[2]=fh->vertex(2)->point();
	if(check_inside(query,points,points+3, K())){
		cout << "Found it in "<< traversed << " checks"  <<  endl;
		cout << "Triangle: " << fh->vertex(0)->point() <<" | " 
								<< fh->vertex(1)->point() <<" | " 
								<< fh->vertex(2)->point() <<endl ;
		return true;
	}
//	else
//		cout << "Not found in here.. " << endl;
	for(int i=0;i<3;i++)
		if(fh->neighbor(i)->info().visited==false && fh->neighbor(i)->info().in_domain())
			if(locate_in_neighbourhood(cdt,fh->neighbor(i),query,traversed+1)){
				cout << "From: " << fh->neighbor(i)->vertex(0)->point() <<" | " 
								<< fh->neighbor(i)->vertex(1)->point() <<" | " 
								<< fh->neighbor(i)->vertex(2)->point() <<endl ;
				return true;
			}
}	

Polygon_2 read_from_file(string filename){

	ifstream input(filename.c_str());
	double a, b;
	Polygon_2 npol;
	while (input >> a >> b)
	{
		npol.push_back(Point(a,b));
	}
	input.close();
	
	return npol;
}

void write_to_file(string filename,Polygon_2& pol){

	ofstream output;
	output.open(filename.c_str());
	
	for (Polygon_2::Vertex_iterator vit=pol.vertices_begin();
		vit!=pol.vertices_end();++vit){
		
		output << vit->x() << "\t" << vit->y() << endl;
		
	}
	output.close();
}

void write_to_file(CDT& cdt){
	
	ofstream output;
	output.open("./data/output/triangulation.csv");
	ofstream output1;
	output1.open("./data/output/triangulation_extra.csv");
	for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin();
                          fit!=cdt.finite_faces_end();++fit){
		if ( fit->info().in_domain() ){
			output << fit->vertex(0)->point().x() << "\t" << fit->vertex(0)->point().y() << endl;
			output << fit->vertex(1)->point().x() << "\t" << fit->vertex(1)->point().y() << endl;
			output << fit->vertex(2)->point().x() << "\t" << fit->vertex(2)->point().y() << endl;
			output << fit->vertex(0)->point().x() << "\t" << fit->vertex(0)->point().y() << endl;
			output << endl;
		}
		output1 << fit->vertex(0)->point().x() << "\t" << fit->vertex(0)->point().y() << endl;
		output1 << fit->vertex(1)->point().x() << "\t" << fit->vertex(1)->point().y() << endl;
		output1 << fit->vertex(2)->point().x() << "\t" << fit->vertex(2)->point().y() << endl;
		output1 << fit->vertex(0)->point().x() << "\t" << fit->vertex(0)->point().y() << endl;
		output1 << endl;
	}
	output.close();
	output1.close();
}

int main()
{
	list <Polygon_2> obstacles;
	DIR *dir;
	struct dirent *ent;
	
	CDT cdt;
	
	if ((dir = opendir ("./data/input/obstacles/")) != NULL) {
		while ((ent = readdir (dir)) != NULL) {
			if( strcmp(ent->d_name,".")==0 || strcmp(ent->d_name,"..")==0)
				continue;
				Polygon_2 temp=read_from_file((string)"./data/input/obstacles/"+(string)ent->d_name);
			obstacles.push_back(temp);
		}
		closedir (dir);
	}
	
	Polygon_2 polygon1=read_from_file("./data/input/b_box.csv");  	
	write_to_file("./data/output/b_box.csv",polygon1);
  	
	cout << "Found " << obstacles.size() << " obstacles" << endl;
	
	double d1=0;
	double d2=0;
	
	cout << "Please insert the coordinates of the starting point" << endl;
	cin >> d1 >> d2 ;
//	Point query2(1.4,1.3);
	Point query(d1,d2);
	
	
	cout << "Please insert the coordinates of the ending point" << endl;
	cin >> d1 >> d2 ;
//	Point query(1.1,0.7);
	Point query2(d1,d2);
	
   	insert_polygon(cdt,polygon1);
   	
   	int i=0;
   	for(list<Polygon_2>::iterator it=obstacles.begin();
		it!=obstacles.end();++it){
		insert_polygon(cdt,*it);
		string filename="./data/output/";
		filename+= (string) "polygon_";
		char temp[10];
		memset(temp,'\0',10*sizeof(char));
		sprintf(temp,"%d",i);
		filename+= (string) temp;
		filename+= (string) ".csv";
		write_to_file(filename,*it);
		i++;
	}
  
	mark_domains(cdt);
  
  	write_to_file(cdt);
	  
	Face_handle fh = cdt.locate(query);

	list <pathNode> pathList;
	if(!find_paths(cdt,fh,query2,&pathList))
		cout << "No path was found! " << endl;
	else
		cout << "The path is printed.. (Order: END -> START) "<< endl;
		
	return 0;
}

