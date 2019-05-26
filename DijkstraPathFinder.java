import java.util.*;

public class DijkstraPathFinder implements PathFinder
{
	private PathMap map;
	private int[][] tcost=null;//tentative costs
	private Coordinate[][] prevc=null;//previous cell array
	private int coordinatesExplored=0;
	
    public DijkstraPathFinder(PathMap map) 
    {
    	this.map=map;
    	initCostsAndPrev(true);
    }
    //initialise tentative costs and previous cell arrays
    //if setOriginsZero is true then sets origin cell tentative costs to 0
    private void initCostsAndPrev(boolean setOriginsZero)
    {
    	if(tcost==null)
    		tcost=new int[map.sizeR][map.sizeC];
    	for(int i=0;i<map.sizeR;++i)
    		for(int j=0;j<map.sizeC;++j)
    			tcost[i][j]=Integer.MAX_VALUE;

    	if(setOriginsZero)
    		for(Coordinate c : map.originCells)
    			tcost[c.getRow()][c.getColumn()]=0;
    	
    	if(prevc==null)
    		prevc=new Coordinate[map.sizeR][map.sizeC];
    	for(int i=0;i<map.sizeR;++i)
    		for(int j=0;j<map.sizeC;++j)
    			prevc[i][j]=null;
    }

    @Override
    //finds the shortest path regardless of number/combination of waypoints, origins and destinations
    public List<Coordinate> findPath() 
    {
    	List<Coordinate> path=new ArrayList<Coordinate>();

    	//if we have waypoints
    	if(map.waypointCells.size()!=0)
    	{
    		//get intermediate paths to waypoints and add to path
    		for(Coordinate c : map.waypointCells)
    		{
    			runAlgorithm(c);
    			if(path.size()>0)
    				path.remove(path.size()-1);
    			path.addAll(getShortestPath(c));
    			initCostsAndPrev(false);
    			tcost[c.getRow()][c.getColumn()]=0;
    		}
    		if(path.size()>0)
    			path.remove(path.size()-1);
    		path.addAll(getShortestPath(map.waypointCells.get(map.waypointCells.size()-1)));
    		
    		
    		initCostsAndPrev(false);
    		Coordinate c=map.waypointCells.get(map.waypointCells.size()-1);
			tcost[c.getRow()][c.getColumn()]=0;//set last waypoint as origin
			//set null to get routes for all destinations
			runAlgorithm(null);
			//find closest destination from last waypoint and add to path
			List<Coordinate> dpath=null;
			int dcost=Integer.MAX_VALUE;
			for(int i=0;i<map.destCells.size();++i)
			{
				List<Coordinate> tpath=getShortestPath(map.destCells.get(i));
				if(dpath==null || getPathCost(tpath)<dcost)
				{
					dpath=tpath;
					dcost=getPathCost(tpath);
				}
			}
			if(path.size()>0)
    			path.remove(path.size()-1);
    		path.addAll(dpath);
    	}
    	else//no waypoints
    	{
    		if(map.destCells.size()==1)//specify destination to stop algorithm once reached
    			runAlgorithm(map.destCells.get(0));
    		else
    			runAlgorithm(null);//null to find all destinations
    		//find shortest route
    		int minDistance = Integer.MAX_VALUE;
    		for(Coordinate c : map.destCells)
    		{
    			List<Coordinate> temp=getShortestPath(c);
    			if(getPathCost(temp)<minDistance)
    			{
    				minDistance=getPathCost(temp);
    				path=temp;
    			}
    		}
    	}
        return path;
    }
    //this algorithm based on pseudocode from https://en.wikipedia.org/wiki/Dijkstra's_algorithm
    private void runAlgorithm(Coordinate dest)
    {
    	List<Coordinate> unvisited=getUnvisited();
        
    	while(unvisited.size()>0)
    	{
    		Coordinate curr=getLowestDistance(unvisited);
    		unvisited.remove(curr);
    		coordinatesExplored++;
    		if(dest!=null && curr.equals(dest))
    		{
    			System.out.println("breaking");
    			break;
    		}
    		List<Coordinate> nbrs=notVisited(unvisited,getNeighbours(curr));
    		for(Coordinate n : nbrs)
    		{
    			int cost=tcost[curr.getRow()][curr.getColumn()]+map.cells[n.getRow()][n.getColumn()].getTerrainCost();
    			if(cost<tcost[n.getRow()][n.getColumn()])
    			{
    				tcost[n.getRow()][n.getColumn()]=cost;
    				prevc[n.getRow()][n.getColumn()]=curr;
    			}
    		}
    	}
    }
    //calculates path cost
    private int getPathCost(List<Coordinate> path)
    {
    	int cost=0;
    	for(Coordinate c: path)
    		cost+=map.cells[c.getRow()][c.getColumn()].getTerrainCost();

    	return cost;
    }
    // returns shortest path to specified destination by looking up prevc array for each coord
    private List<Coordinate> getShortestPath(Coordinate destination)
    {
    	List<Coordinate> reverse = new ArrayList<Coordinate>();
    	
    	Coordinate curr=destination;
    	while(curr!=null)
    	{
    		reverse.add(curr);
    		curr=prevc[curr.getRow()][curr.getColumn()];
    	}
    	List<Coordinate> path=new ArrayList<Coordinate>();
    	for(int i=reverse.size()-1;i>=0;i--)
    		path.add(reverse.get(i));
        return path;
    }
    //returns the coord with lowest distance/cost out of the given unvisited list
    private Coordinate getLowestDistance(List<Coordinate> unvisited)
    {
    	Coordinate curr=unvisited.get(0);
    	for(Coordinate c : unvisited)
    		if(tcost[c.getRow()][c.getColumn()]<tcost[curr.getRow()][curr.getColumn()])
    			curr=c;
    	return curr;
    }
    //returns a list containing all passable coords in the map
    private List<Coordinate> getUnvisited()
	{
    	List<Coordinate> unvisited=new ArrayList<Coordinate>();
    	for(int i=0;i<map.sizeR;++i)
    		for(int j=0;j<map.sizeC;++j)
    		{
    			Coordinate c=map.cells[i][j];
    			if(map.isPassable(i, j))
    				unvisited.add(c);
    		}
   		return unvisited;
	}
    //returns a list containing the coords that havent been visited yet
    private List<Coordinate> notVisited(List<Coordinate> unvisited, List<Coordinate> coords)
    {
    	List<Coordinate> nvc=new ArrayList<Coordinate>();
    	for(int i=0;i<coords.size();++i)
    	{
    		Coordinate cp = coords.get(i);
    		if(unvisited.contains(cp))
    			nvc.add(cp);
    	}
    	return nvc;
    }
    //returns a list containing the (upto)4 neighbours of the specified coordinate
    private List<Coordinate> getNeighbours(Coordinate coord)
    {
    	List<Coordinate> neighbours = new ArrayList<Coordinate>();
    	
    	//add above
    	if(coord.getRow()>0 && map.isPassable(coord.getRow()-1,coord.getColumn()))
    		neighbours.add(map.cells[coord.getRow()-1][coord.getColumn()]);
    	//add below
    	if(coord.getRow()<map.sizeR-1 && map.isPassable(coord.getRow()+1,coord.getColumn()))
    		neighbours.add(map.cells[coord.getRow()+1][coord.getColumn()]);
    	//add left
    	if(coord.getColumn()>0 && map.isPassable(coord.getRow(),coord.getColumn()-1))
    		neighbours.add(map.cells[coord.getRow()][coord.getColumn()-1]);
    	//add right
    	if(coord.getColumn()<map.sizeC-1 && map.isPassable(coord.getRow(),coord.getColumn()+1))
    		neighbours.add(map.cells[coord.getRow()][coord.getColumn()+1]);
    	
    	return neighbours;
    }

    @Override
    public int coordinatesExplored() 
    {
        return coordinatesExplored;
    }
}
