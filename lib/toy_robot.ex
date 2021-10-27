defmodule ToyRobot do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  defstruct x: 1, y: 1, facing: :north, walkable: true, gcost: 0,hcost: 10,parent: Elixir.ToyRobot


  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> ToyRobot.place
      {:ok, %ToyRobot.Position{facing: :north, x: 1, y: :a}}
  """

  def place do
    {:ok, %ToyRobot.Position{}}
  end

  def place(x, y, _facing) when x < 1 or y < :a or x > @table_top_x or y > @table_top_y do
    {:failure, "Invalid position"}
  end

  def place(_x, _y, facing)
  when facing not in [:north, :east, :south, :west]
  do
    {:failure, "Invalid facing direction"}
  end

  @doc """
  Places the robot to the provided position of (x, y, facing),
  but prevents it to be placed outside of the table and facing invalid direction.

  Examples:

      iex> ToyRobot.place(1, :b, :south)
      {:ok, %ToyRobot.Position{facing: :south, x: 1, y: :b}}

      iex> ToyRobot.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> ToyRobot.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    {:ok, %ToyRobot.Position{x: x, y: y, facing: facing}}
  end


  def move_to(n,robot,cli_proc_name) when n <= 0 do
    robot
  end

  def move_to(n,robot,cli_proc_name)do
    send_robot_status(robot,cli_proc_name)
    robot = move(robot)
    move_to(n-1,robot,cli_proc_name)
  end


  #loop for orientation
  def rotate(robot, required) when robot.facing == required do
      # IO.puts(robot.facing)
      robot
  end

  def rotate(robot, required) do
      robot = left(robot)
      robot = rotate(robot, required)
  end


  def change_orientation_x(robot,error_x) do

    robot = if(error_x>0) do
      rotate(robot, :east)
    else
      robot
    end

    robot = if(error_x<0) do
      rotate(robot, :west)
    else
      robot
    end

    robot
  end

  def change_orientation_y(robot,error_y) do

    robot = if(error_y>0) do
      rotate(robot, :north)
    else
      robot
    end

    robot = if(error_y<0) do
      rotate(robot, :south)
    else
      robot
    end

    robot
  end


  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do

    place(x, y, facing)

  end


  def go_to_point(robot,goal_x, goal_y,cli_proc_name) do

    #IO.inspect robot ,label: "robot is "

    error_x = goal_x-robot.x
    error_y = goal_y-@robot_map_y_atom_to_num[robot.y]

  robot = if(error_y != 0) do
    robot = change_orientation_y(robot, error_y)

    robot = if(error_y<0) do
      move_to(-1*error_y,robot,cli_proc_name)
    else
      move_to(error_y,robot,cli_proc_name)
    end

  else

    robot = change_orientation_x(robot, error_x)
    robot = if(error_x<0) do
      move_to(-1*error_x,robot,cli_proc_name)
    else
      move_to(error_x,robot,cli_proc_name)
    end
  end
    robot

  end

  def obstacle_found(is_obstacle) do
    IO.puts(is_obstacle)
  end


  def path_loop(currentNode, startNode, path) when (currentNode.x==startNode.x and currentNode.y==startNode.y )do
    path
  end

  def path_loop(currentNode, startNode, path) do
    path = path ++ [currentNode]
    newNode = currentNode.parent

    x = newNode.x
    y= newNode.y
    parent =newNode.parent
    currentNode = %{currentNode | x: x, y: y,parent: parent}

    path = path_loop(currentNode,startNode,path)

  end


  def retracePath(startNode, endNode) do

    path = []
		currentNode = endNode

    path = path_loop(currentNode, startNode, path)

		path = Enum.reverse(path)
    path
  end


  def getNeighbours(node1) do
    #IO.puts "Get neighbours called
    neighbours = []

    #call server -true ,false

    #north
    temp = node1.y + 1
    neighbours = if (node1.x > 0 and node1.x <= 5 and temp > 0 and temp <= 5) do
      neighbours ++ [%ToyRobot{x: node1.x, y: temp}]
    else
      neighbours
    end

    #south
    temp = node1.y - 1
    neighbours = if (node1.x > 0 and node1.x <= 5 and temp > 0 and temp <= 5) do
      neighbours = neighbours ++ [%ToyRobot{x: node1.x, y: temp}]
    else
      neighbours
    end

    #east
    temp = node1.x + 1
    neighbours = if (node1.y > 0 and node1.y <= 5 and temp > 0 and temp <= 5) do
      neighbours ++ [%ToyRobot{x: temp, y: node1.y}]
    else
      neighbours
    end

    #west
    temp = node1.x - 1
    neighbours = if (node1.y > 0 and node1.y <= 5 and temp > 0 and temp <= 5) do
      neighbours ++ [%ToyRobot{x: temp, y: node1.y}]
    else
      neighbours
    end

    {:ok,neighbours}
  end

  def getDistance(nodeA, nodeB) do
      #IO.puts "Get distance called"
      dstX = abs(nodeA.x - nodeB.x);
      dstY = abs(nodeA.y - nodeB.y);

      result = if (dstX > dstY) do
        14*dstY + 10* (dstX-dstY);

      else
        14*dstX + 10 * (dstY-dstX);
      end
  end


  @spec astar_inner_loop_2(any, any, any, any, any, any, any) :: {:ok, [...]}
  def astar_inner_loop_2(x, n, neighbours, closedset, openset, node1, targetNode) when x==n do
    {:ok,[ neighbours, closedset, openset, node1]}
  end

  def astar_inner_loop_2(x, n, neighbours, closedset, openset, node1, targetNode) do
        #IO.puts "Astar inner loop 2"
        currentNode = Enum.at(neighbours,x)

				if (!currentNode.walkable or Enum.member?(closedset, currentNode)) do
					astar_inner_loop_2(x+1, n, neighbours, closedset, openset, node1, targetNode)

        else

				newCostToNeighbour = node1.gcost + getDistance(node1, currentNode)

				openset = if (newCostToNeighbour < currentNode.gcost || !Enum.member?(openset, currentNode)) do
          distance = getDistance(currentNode, targetNode)

          currentNode = %{ currentNode | gcost: newCostToNeighbour, hcost: distance, parent: node1}

					if (!Enum.member?(openset, currentNode)) do
						openset ++ [currentNode]
          end
        end
        {:ok,[ neighbours, closedset, openset, node1]} = astar_inner_loop_2(x+1, n, neighbours, closedset, openset, node1, targetNode)
      end
  end



  def astar_inner_loop(openset,closedset,startNode, node1,targetNode, x, n) when x==n do
    openset = List.delete(openset, node1)
    closedset = closedset ++ [node1]
    completed = false
    # IO.inspect closedset, label: "The closedset is"


    # IO.inspect(node1)
    # IO.inspect(targetNode)

			path  = if (node1.x==targetNode.x and node1.y==targetNode.y) do
        retracePath(startNode,node1)
      else
        []
      end

      completed = if (node1.x==targetNode.x and node1.y==targetNode.y) do
        true
      else
        false
      end

    {:ok,[path, node1,openset,closedset,completed]}

  end

  def astar_inner_loop(openset, closedset,startNode, node1,targetNode, x, n) do

    #IO.inspect node1, label: "this is node1 inner loop"

    fcost_openset = Enum.at(openset, x).gcost + Enum.at(openset, x).hcost
    fcost_node = node1.gcost + node1.hcost

    #IO.puts "Astar inner loop 1"
    node1 = if ((fcost_openset < fcost_node) or (fcost_openset == fcost_node)) do
      node1 = if (fcost_openset < fcost_node) do
        node1 = Enum.at(openset, x)
      else
        node1
      end
    else
      node1
    end
    # IO.inspect openset, label: "The openset is"
    {:ok,[path, node1,openset,closedset,completed]} = astar_inner_loop(openset,closedset, startNode, node1,targetNode, x+1, n)

  end


  def astar_loop1(n, openset, closedset, x_goal, y_goal, startNode,path) when n<1 do
    path
  end

  def astar_loop1(n, openset, closedset, x_goal, y_goal, startNode,path) do

    node1 = hd(openset)
    #IO.inspect node1, label: "this is node1"
    targetNode = %ToyRobot{x: x_goal, y: @robot_map_y_atom_to_num[y_goal]}

    #IO.puts "Astar loop1"

    {:ok,[path, node1,openset,closedset,completed]} = astar_inner_loop(openset,closedset,startNode, node1,targetNode, 1, length(openset))

    path = if(completed) do
      astar_loop1(0, openset, closedset, x_goal, y_goal, startNode,path)
    else

    {:ok,neighbours}  = getNeighbours(node1)
    {:ok,[ neighbours, closedset, openset, node1]} = astar_inner_loop_2(0, length(neighbours), neighbours, closedset, openset, node1, targetNode)

    astar_loop1(length(openset), openset, closedset, x_goal, y_goal, startNode,path)
    end
  end


  def traverse_path_nodes(x, n, robot,cli_proc_name,path) when x==n do
    robot
  end

  def traverse_path_nodes(x, n, robot,cli_proc_name,path) do

    currentNode = Enum.at(path,x)

    robot = go_to_point(robot,currentNode.x,currentNode.y,cli_proc_name)

    traverse_path_nodes(x+1, n, robot,cli_proc_name,path)
  end




  def stop(_robot, goal_x, goal_y, _cli_proc_name) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide STOP position to the robot as given location of (x, y) and plan the path from START to STOP.
  Passing the CLI Server process name that will be used to send robot's current status after each action is taken.
  """

  def stop(robot, goal_x, goal_y, cli_proc_name) do

    Process.register(spawn_link(fn -> listen_from_server() end), :client_toyrobot)

    #go_to_point(robot, goal_x, goal_y,cli_proc_name)

    startNode = %ToyRobot{x: robot.x, y: @robot_map_y_atom_to_num[robot.y]}
    path = []
    openset = []
    closedset = []

    openset = openset ++ [startNode]

    path = astar_loop1(length(openset), openset, closedset, goal_x, goal_y,startNode,path)

    #IO.inspect path,label: "The path is"

    robot = traverse_path_nodes(0, length(path),robot,cli_proc_name,path)

  end

  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the CLI Server process after each action is taken.
  """
  def send_robot_status(%ToyRobot.Position{x: x, y: y, facing: facing} = _robot, cli_proc_name) do
    send(cli_proc_name,{:toyrobot_status, x, y, facing})
    # IO.puts("Sent by Toy Robot Client: #{x}, #{y}, #{facing}")
  end

  @doc """
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def listen_from_server() do
    receive do
      {:obstacle_presence, is_obs_ahead} ->
        if(is_obs_ahead) do
          obstacle_found(is_obs_ahead)
        end
        is_obs_ahead
    end
    listen_from_server()
  end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = ToyRobot.place(2, :b, :west)
      iex> ToyRobot.report(robot)
      {2, :b, :west}
  """
  def report(%ToyRobot.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right.
  """
  def right(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left.
  """
  def left(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall.
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the east, but prevents it to fall.
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %ToyRobot.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall.
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall.
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %ToyRobot.Position{robot | x: x - 1}
  end

  @doc """
  Does not change the position of the robot.
  This function used as fallback if the robot cannot move outside the table.
  """
  def move(robot), do: robot

  def failure do
    raise "Connection has been lost"
  end
end
