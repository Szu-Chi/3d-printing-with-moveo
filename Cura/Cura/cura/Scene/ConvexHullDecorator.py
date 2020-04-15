# Copyright (c) 2016 Ultimaker B.V.
# Cura is released under the terms of the LGPLv3 or higher.
## Find_moveo ##

from PyQt5.QtCore import QTimer

from UM.Application import Application
from UM.Math.Polygon import Polygon

from UM.Scene.SceneNodeDecorator import SceneNodeDecorator
from UM.Settings.ContainerRegistry import ContainerRegistry

from cura.Settings.ExtruderManager import ExtruderManager
from cura.Scene import ConvexHullNode

import numpy

from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    from UM.Scene.SceneNode import SceneNode
    from cura.Settings.GlobalStack import GlobalStack
    from UM.Mesh.MeshData import MeshData
    from UM.Math.Matrix import Matrix

from UM.Math.Quaternion import Quaternion
from UM.Operations.SetTransformOperation import SetTransformOperation
from UM.Operations.GroupedOperation import GroupedOperation
from UM.Math.Vector import Vector
import math

import threading
from queue import Queue

##  The convex hull decorator is a scene node decorator that adds the convex hull functionality to a scene node.
#   If a scene node has a convex hull decorator, it will have a shadow in which other objects can not be printed.
class ConvexHullDecorator(SceneNodeDecorator):
    def __init__(self) -> None:
        super().__init__()

        self._seve_world_position_x = None
        self._seve_world_position_y = None
        self._seve_world_position_z = None
        self._rotate_mesh_convexhull = None
        self._rotate_world_transform = None
        self._convex_hull_node = None  # type: Optional["SceneNode"]
        self._init2DConvexHullCache()

        self._global_stack = None  # type: Optional[GlobalStack]

        # Make sure the timer is created on the main thread
        self._recompute_convex_hull_timer = None  # type: Optional[QTimer]
        from cura.CuraApplication import CuraApplication
        if CuraApplication.getInstance() is not None:
            CuraApplication.getInstance().callLater(self.createRecomputeConvexHullTimer)

        self._raft_thickness = 0.0
        self._build_volume = CuraApplication.getInstance().getBuildVolume()
        self._build_volume.raftThicknessChanged.connect(self._onChanged)

        CuraApplication.getInstance().globalContainerStackChanged.connect(self._onGlobalStackChanged)
        CuraApplication.getInstance().getController().toolOperationStarted.connect(self._onChanged)
        CuraApplication.getInstance().getController().toolOperationStopped.connect(self._onChanged)

        self._onGlobalStackChanged()

    def createRecomputeConvexHullTimer(self) -> None:
        self._recompute_convex_hull_timer = QTimer()
        self._recompute_convex_hull_timer.setInterval(200)
        self._recompute_convex_hull_timer.setSingleShot(True)
        self._recompute_convex_hull_timer.timeout.connect(self.recomputeConvexHull)

    def setNode(self, node: "SceneNode") -> None:
        previous_node = self._node
        # Disconnect from previous node signals
        if previous_node is not None and node is not previous_node:
            previous_node.boundingBoxChanged.disconnect(self._onChanged)

        super().setNode(node)

        node.boundingBoxChanged.connect(self._onChanged)

        per_object_stack = node.callDecoration("getStack")
        if per_object_stack:
            per_object_stack.propertyChanged.connect(self._onSettingValueChanged)

        self._onChanged()

    ## Force that a new (empty) object is created upon copy.
    def __deepcopy__(self, memo):
        return ConvexHullDecorator()

    ## The polygon representing the 2D adhesion area.
    # If no adhesion is used, the regular convex hull is returned
    def getAdhesionArea(self) -> Optional[Polygon]:
        if self._node is None:
            return None

        hull = self._compute2DConvexHull()
        if hull is None:
            return None

        return self._add2DAdhesionMargin(hull)

    ##  Get the unmodified 2D projected convex hull with 2D adhesion area of the node (if any)
    def getConvexHull(self) -> Optional[Polygon]:
        if self._node is None:
            return None
        if self._node.callDecoration("isNonPrintingMesh"):
            return None
        hull = self._compute2DConvexHull()
        if self._global_stack and self._node is not None and hull is not None:
            # Parent can be None if node is just loaded.
            if self._global_stack.getProperty("print_sequence", "value") == "one_at_a_time" and not self.hasGroupAsParent(self._node):
                hull = hull.getMinkowskiHull(Polygon(numpy.array(self._global_stack.getProperty("machine_head_polygon", "value"), numpy.float32)))
                hull = self._add2DAdhesionMargin(hull)
        return hull

    def getConvexHullForMoveo(self) -> Optional[Polygon]:
        if self._node is None:
            return None
        if self._node.callDecoration("isNonPrintingMesh"):
            return None
        world_transform = self._node.getWorldTransformation()
        rotate_mesh = self._node.getMeshData()
        rotate_mesh_convexhull = rotate_mesh.getConvexHullTransformedVertices(world_transform)
        # If change position or rotation, we have to find new convexhull
        if numpy.array_equal(self._rotate_mesh_convexhull,rotate_mesh_convexhull) or self._seve_world_position_x != self._node.getWorldPosition().x or self._seve_world_position_y != self._node.getWorldPosition().y or self._seve_world_position_z != self._node.getWorldPosition().z:
            # Save data
            self._seve_world_position_x = self._node.getWorldPosition().x
            self._seve_world_position_y = self._node.getWorldPosition().y
            self._seve_world_position_z = self._node.getWorldPosition().z
            self._rotate_mesh_convexhull = rotate_mesh.getConvexHullTransformedVertices(world_transform)

            num = -1
            hypotenuse = 0
            # Find the largest hypotenuse of all convexhull (drop z)
            for i in range(1,numpy.size(self._rotate_mesh_convexhull,0)):
                if hypotenuse < ((self._rotate_mesh_convexhull[i][0]**2 + self._rotate_mesh_convexhull[i][2]**2)**0.5):
                    hypotenuse = (self._rotate_mesh_convexhull[i][0]**2 + self._rotate_mesh_convexhull[i][2]**2)**0.5
                    num = i
            # Find angle of the largest hypotenuse
            angle = math.asin(self._rotate_mesh_convexhull[num][0]/hypotenuse)
            if self._node.getWorldPosition().x == 0.0 and self._node.getWorldPosition().z > 0:
                angle = math.pi
            else:    
                angle = angle if self._node.getWorldPosition().z < 0 else angle*(math.pi/abs(angle)-1)
            # Set rotate matrix
            rotate_and_shift_to_Y_matrix = numpy.array([[ math.cos(angle), 0, math.sin(angle), 0],
                                                        [               0, 1,               0, 0],
                                                        [-math.sin(angle), 0, math.cos(angle), 0],
                                                        [               0, 0,               0, 1]])
            # Rotate all convexhull
            one_column = numpy.ones([1,numpy.size(self._rotate_mesh_convexhull,0)])
            shift_to_o = numpy.column_stack((self._rotate_mesh_convexhull,one_column.T))
            save = numpy.dot(rotate_and_shift_to_Y_matrix,shift_to_o[0])
            for j in range(1,numpy.size(self._rotate_mesh_convexhull,0)):
                shift_to_Y = numpy.dot(rotate_and_shift_to_Y_matrix,shift_to_o[j])
                save = numpy.row_stack((save,shift_to_Y))
            self._rotate_mesh_convexhull = numpy.delete(save,-1,axis=1)

        # Use threads to speedup code
        q = Queue()
        threads = []

        for i in range(2):
            threads.append(threading.Thread(target=self._compute2DConvexHullForMoveo, args=(i+1, q)))
            threads[i].start()

        for thread in threads:
            thread.join()

        results = []
        for _ in range(2):
            hull = q.get()
            if self._global_stack and self._node is not None and hull is not None:
            # Parent can be None if node is just loaded.
                if self._global_stack.getProperty("print_sequence", "value") == "one_at_a_time" and not self.hasGroupAsParent(self._node):
                    hull = hull.getMinkowskiHull(Polygon(numpy.array(self._global_stack.getProperty("machine_head_polygon", "value"), numpy.float32)))
                    hull = self._add2DAdhesionMargin(hull)
            results.append(hull)
        return results

    ##  Get the convex hull of the node with the full head size
    def getConvexHullHeadFull(self) -> Optional[Polygon]:
        if self._node is None:
            return None

        return self._compute2DConvexHeadFull()

    @staticmethod
    def hasGroupAsParent(node: "SceneNode") -> bool:
        parent = node.getParent()
        if parent is None:
            return False
        return bool(parent.callDecoration("isGroup"))

    ##  Get convex hull of the object + head size
    #   In case of printing all at once this is the same as the convex hull.
    #   For one at the time this is area with intersection of mirrored head
    def getConvexHullHead(self) -> Optional[Polygon]:
        if self._node is None:
            return None
        if self._node.callDecoration("isNonPrintingMesh"):
            return None
        if self._global_stack:
            if self._global_stack.getProperty("print_sequence", "value") == "one_at_a_time" and not self.hasGroupAsParent(self._node):
                head_with_fans = self._compute2DConvexHeadMin()
                if head_with_fans is None:
                    return None
                head_with_fans_with_adhesion_margin = self._add2DAdhesionMargin(head_with_fans)
                return head_with_fans_with_adhesion_margin
        return None

    ##  Get convex hull of the node
    #   In case of printing all at once this is the same as the convex hull.
    #   For one at the time this is the area without the head.
    def getConvexHullBoundary(self) -> Optional[Polygon]:
        if self._node is None:
            return None
        
        if self._node.callDecoration("isNonPrintingMesh"):
            return None

        if self._global_stack:
            if self._global_stack.getProperty("print_sequence", "value") == "one_at_a_time" and not self.hasGroupAsParent(self._node):
                # Printing one at a time and it's not an object in a group
                return self._compute2DConvexHull()
        return None

    ##  The same as recomputeConvexHull, but using a timer if it was set.
    def recomputeConvexHullDelayed(self) -> None:
        if self._recompute_convex_hull_timer is not None:
            self._recompute_convex_hull_timer.start()
        else:
            self.recomputeConvexHull()

    def recomputeConvexHull(self) -> None:
        controller = Application.getInstance().getController()
        root = controller.getScene().getRoot()
        if self._node is None or controller.isToolOperationActive() or not self.__isDescendant(root, self._node):
            # If the tool operation is still active, we need to compute the convex hull later after the controller is
            # no longer active.
            if controller.isToolOperationActive():
                self.recomputeConvexHullDelayed()
                return

            if self._convex_hull_node:
                self._convex_hull_node.setParent(None)
                self._convex_hull_node = None
            return

        convex_hull = self.getConvexHull()
        if self._convex_hull_node:
            self._convex_hull_node.setParent(None)
        hull_node = ConvexHullNode.ConvexHullNode(self._node, convex_hull, self._raft_thickness, root)
        self._convex_hull_node = hull_node

    def _onSettingValueChanged(self, key: str, property_name: str) -> None:
        if property_name != "value":  # Not the value that was changed.
            return

        if key in self._affected_settings:
            self._onChanged()
        if key in self._influencing_settings:
            self._init2DConvexHullCache()  # Invalidate the cache.
            self._onChanged()

    def _init2DConvexHullCache(self) -> None:
        # Cache for the group code path in _compute2DConvexHull()
        self._2d_convex_hull_group_child_polygon = None  # type: Optional[Polygon]
        self._2d_convex_hull_group_result = None  # type: Optional[Polygon]

        # Cache for the mesh code path in _compute2DConvexHull()
        self._2d_convex_hull_mesh = None  # type: Optional[MeshData]
        self._2d_convex_hull_mesh_world_transform = None  # type: Optional[Matrix]
        self._2d_convex_hull_mesh_result = None  # type: Optional[Polygon]

    def eulerToQuaternion(self, roll, pitch, yaw):
        quaternion_x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        quaternion_y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        quaternion_z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        quaternion_w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [quaternion_x,quaternion_y,quaternion_z,quaternion_w]

    def quaternionToEuler(self, quaternion_x, quaternion_y, quaternion_z, quaternion_w):
        roll = math.atan2(2.0*(quaternion_w*quaternion_x+quaternion_y*quaternion_z),1.0-2.0*(quaternion_x*quaternion_x+quaternion_y*quaternion_y))
        pitch = math.asin(2.0*(quaternion_w*quaternion_y-quaternion_z*quaternion_x))
        yaw = math.atan2(2.0*(quaternion_w*quaternion_z+quaternion_x*quaternion_y),1.0-2.0*(quaternion_z*quaternion_z+quaternion_y*quaternion_y))
        return [roll,pitch,yaw]

    def _compute2DConvexHull(self) -> Optional[Polygon]:
        if self._node is None:
            return None
        if self._node.callDecoration("isGroup"):
            points = numpy.zeros((0, 2), dtype=numpy.int32)
            for child in self._node.getChildren():
                child_hull = child.callDecoration("_compute2DConvexHull")
                if child_hull:
                    try:
                        points = numpy.append(points, child_hull.getPoints(), axis = 0)
                    except ValueError:
                        pass

                if points.size < 3:
                    return None
            child_polygon = Polygon(points)

            # Check the cache
            if child_polygon == self._2d_convex_hull_group_child_polygon:
                return self._2d_convex_hull_group_result

            convex_hull = child_polygon.getConvexHull() #First calculate the normal convex hull around the points.
            offset_hull = self._offsetHull(convex_hull) #Then apply the offset from the settings.

            # Store the result in the cache
            self._2d_convex_hull_group_child_polygon = child_polygon
            self._2d_convex_hull_group_result = offset_hull

            return offset_hull

        else:
            offset_hull = Polygon([])
            mesh = self._node.getMeshData()
            if mesh is None:
                return Polygon([])  # Node has no mesh data, so just return an empty Polygon.

            world_transform = self._node.getWorldTransformation()

            # Check the cache
            if mesh is self._2d_convex_hull_mesh and world_transform == self._2d_convex_hull_mesh_world_transform:
                return self._2d_convex_hull_mesh_result

            vertex_data = mesh.getConvexHullTransformedVertices(world_transform)
            # Don't use data below 0.
            # TODO; We need a better check for this as this gives poor results for meshes with long edges.
            # Do not throw away vertices: the convex hull may be too small and objects can collide.
            # vertex_data = vertex_data[vertex_data[:,1] >= -0.01]

            if len(vertex_data) >= 4:  # type: ignore # mypy and numpy don't play along well just yet.
                # Round the vertex data to 1/10th of a mm, then remove all duplicate vertices
                # This is done to greatly speed up further convex hull calculations as the convex hull
                # becomes much less complex when dealing with highly detailed models.
                vertex_data = numpy.round(vertex_data, 1)

                vertex_data = vertex_data[:, [0, 2]]  # Drop the Y components to project to 2D.

                # Grab the set of unique points.
                #
                # This basically finds the unique rows in the array by treating them as opaque groups of bytes
                # which are as long as the 2 float64s in each row, and giving this view to numpy.unique() to munch.
                # See http://stackoverflow.com/questions/16970982/find-unique-rows-in-numpy-array
                vertex_byte_view = numpy.ascontiguousarray(vertex_data).view(
                    numpy.dtype((numpy.void, vertex_data.dtype.itemsize * vertex_data.shape[1])))
                _, idx = numpy.unique(vertex_byte_view, return_index = True)
                vertex_data = vertex_data[idx]  # Select the unique rows by index.

                hull = Polygon(vertex_data)

                if len(vertex_data) >= 3:
                    convex_hull = hull.getConvexHull()
                    offset_hull = self._offsetHull(convex_hull)

            # Store the result in the cache
            self._2d_convex_hull_mesh = mesh
            self._2d_convex_hull_mesh_world_transform = world_transform
            self._2d_convex_hull_mesh_result = offset_hull

            return offset_hull

    def _compute2DConvexHullForMoveo(self, moveo_check_xz_or_yz = None, q = None) -> Optional[Polygon]:
        if self._node is None:
            q.put(None)
        if self._node.callDecoration("isGroup"):
            points = numpy.zeros((0, 2), dtype=numpy.int32)
            for child in self._node.getChildren():
                child_hull = child.callDecoration("_compute2DConvexHull")
                if child_hull:
                    try:
                        points = numpy.append(points, child_hull.getPoints(), axis = 0)
                    except ValueError:
                        pass

                if points.size < 3:
                    q.put(None)
            child_polygon = Polygon(points)

            # Check the cache
            if child_polygon == self._2d_convex_hull_group_child_polygon:
                q.put(self._2d_convex_hull_group_result)

            convex_hull = child_polygon.getConvexHull() #First calculate the normal convex hull around the points.
            offset_hull = self._offsetHull(convex_hull) #Then apply the offset from the settings.

            # Store the result in the cache
            self._2d_convex_hull_group_child_polygon = child_polygon
            self._2d_convex_hull_group_result = offset_hull
            q.put(offset_hull)
        else:
            offset_hull = Polygon([])

            mesh = self._node.getMeshData()
            if mesh is None:
                q.put(Polygon([]))

            vertex_data = self._rotate_mesh_convexhull
            # Don't use data below 0.
            # TODO; We need a better check for this as this gives poor results for meshes with long edges.
            # Do not throw away vertices: the convex hull may be too small and objects can collide.
            # vertex_data = vertex_data[vertex_data[:,1] >= -0.01]

            if len(vertex_data) >= 4:  # type: ignore # mypy and numpy don't play along well just yet.
                # Round the vertex data to 1/10th of a mm, then remove all duplicate vertices
                # This is done to greatly speed up further convex hull calculations as the convex hull
                # becomes much less complex when dealing with highly detailed models.
                vertex_data = numpy.round(vertex_data, 1)
                if moveo_check_xz_or_yz == 1:
                    vertex_data = vertex_data[:, [0, 1]]  # Drop the Z components to project to 2D.(Y)
                elif moveo_check_xz_or_yz == 2:
                    vertex_data = vertex_data[:, [2, 1]]  # Drop the X components to project to 2D.(X)
                # Grab the set of unique points.
                #
                # This basically finds the unique rows in the array by treating them as opaque groups of bytes
                # which are as long as the 2 float64s in each row, and giving this view to numpy.unique() to munch.
                # See http://stackoverflow.com/questions/16970982/find-unique-rows-in-numpy-array
                vertex_byte_view = numpy.ascontiguousarray(vertex_data).view(
                    numpy.dtype((numpy.void, vertex_data.dtype.itemsize * vertex_data.shape[1])))
                _, idx = numpy.unique(vertex_byte_view, return_index = True)
                vertex_data = vertex_data[idx]  # Select the unique rows by index.

                hull = Polygon(vertex_data)

                if len(vertex_data) >= 3:
                    convex_hull = hull.getConvexHull()
                    offset_hull = self._offsetHull(convex_hull)

            q.put(offset_hull)

    def _getHeadAndFans(self) -> Polygon:
        if not self._global_stack:
            return Polygon()

        polygon = Polygon(numpy.array(self._global_stack.getHeadAndFansCoordinates(), numpy.float32))
        offset_x = self._getSettingProperty("machine_nozzle_offset_x", "value")
        offset_y = self._getSettingProperty("machine_nozzle_offset_y", "value")
        return polygon.translate(-offset_x, -offset_y)

    def _compute2DConvexHeadFull(self) -> Optional[Polygon]:
        convex_hull = self._compute2DConvexHull()
        if convex_hull:
            return convex_hull.getMinkowskiHull(self._getHeadAndFans())
        return None

    def _compute2DConvexHeadMin(self) -> Optional[Polygon]:
        head_and_fans = self._getHeadAndFans()
        mirrored = head_and_fans.mirror([0, 0], [0, 1]).mirror([0, 0], [1, 0])  # Mirror horizontally & vertically.
        head_and_fans = self._getHeadAndFans().intersectionConvexHulls(mirrored)

        # Min head hull is used for the push free
        convex_hull = self._compute2DConvexHull()
        if convex_hull:
            return convex_hull.getMinkowskiHull(head_and_fans)
        return None

    ##  Compensate given 2D polygon with adhesion margin
    #   \return 2D polygon with added margin
    def _add2DAdhesionMargin(self, poly: Polygon) -> Polygon:
        if not self._global_stack:
            return Polygon()
        # Compensate for raft/skirt/brim
        # Add extra margin depending on adhesion type
        adhesion_type = self._global_stack.getProperty("adhesion_type", "value")

        max_length_available = 0.5 * min(
            self._getSettingProperty("machine_width", "value"),
            self._getSettingProperty("machine_depth", "value")
        )

        if adhesion_type == "raft":
            extra_margin = min(max_length_available, max(0, self._getSettingProperty("raft_margin", "value")))
        elif adhesion_type == "brim":
            extra_margin = min(max_length_available, max(0, self._getSettingProperty("brim_line_count", "value") * self._getSettingProperty("skirt_brim_line_width", "value")))
        elif adhesion_type == "none":
            extra_margin = 0
        elif adhesion_type == "skirt":
            extra_margin = min(max_length_available, max(
                0, self._getSettingProperty("skirt_gap", "value") +
                   self._getSettingProperty("skirt_line_count", "value") * self._getSettingProperty("skirt_brim_line_width", "value")))
        else:
            raise Exception("Unknown bed adhesion type. Did you forget to update the convex hull calculations for your new bed adhesion type?")

        # Adjust head_and_fans with extra margin
        if extra_margin > 0:
            extra_margin_polygon = Polygon.approximatedCircle(extra_margin)
            poly = poly.getMinkowskiHull(extra_margin_polygon)
        return poly

    ##  Offset the convex hull with settings that influence the collision area.
    #
    #   \param convex_hull Polygon of the original convex hull.
    #   \return New Polygon instance that is offset with everything that
    #   influences the collision area.
    def _offsetHull(self, convex_hull: Polygon) -> Polygon:
        horizontal_expansion = max(
            self._getSettingProperty("xy_offset", "value"),
            self._getSettingProperty("xy_offset_layer_0", "value")
        )

        mold_width = 0
        if self._getSettingProperty("mold_enabled", "value"):
            mold_width = self._getSettingProperty("mold_width", "value")
        hull_offset = horizontal_expansion + mold_width
        if hull_offset > 0: #TODO: Implement Minkowski subtraction for if the offset < 0.
            expansion_polygon = Polygon(numpy.array([
                [-hull_offset, -hull_offset],
                [-hull_offset, hull_offset],
                [hull_offset, hull_offset],
                [hull_offset, -hull_offset]
            ], numpy.float32))
            return convex_hull.getMinkowskiHull(expansion_polygon)
        else:
            return convex_hull

    def _onChanged(self, *args) -> None:
        self._raft_thickness = self._build_volume.getRaftThickness()
        if not args or args[0] == self._node:
            self.recomputeConvexHullDelayed()

    def _onGlobalStackChanged(self) -> None:
        if self._global_stack:
            self._global_stack.propertyChanged.disconnect(self._onSettingValueChanged)
            self._global_stack.containersChanged.disconnect(self._onChanged)
            extruders = ExtruderManager.getInstance().getActiveExtruderStacks()
            for extruder in extruders:
                extruder.propertyChanged.disconnect(self._onSettingValueChanged)

        self._global_stack = Application.getInstance().getGlobalContainerStack()

        if self._global_stack:
            self._global_stack.propertyChanged.connect(self._onSettingValueChanged)
            self._global_stack.containersChanged.connect(self._onChanged)

            extruders = ExtruderManager.getInstance().getActiveExtruderStacks()
            for extruder in extruders:
                extruder.propertyChanged.connect(self._onSettingValueChanged)

            self._onChanged()

    ##   Private convenience function to get a setting from the correct extruder (as defined by limit_to_extruder property).
    def _getSettingProperty(self, setting_key: str, prop: str = "value") -> Any:
        if self._global_stack is None or self._node is None:
            return None
        per_mesh_stack = self._node.callDecoration("getStack")
        if per_mesh_stack:
            return per_mesh_stack.getProperty(setting_key, prop)

        extruder_index = self._global_stack.getProperty(setting_key, "limit_to_extruder")
        if extruder_index == "-1":
            # No limit_to_extruder
            extruder_stack_id = self._node.callDecoration("getActiveExtruder")
            if not extruder_stack_id:
                # Decoration doesn't exist
                extruder_stack_id = ExtruderManager.getInstance().extruderIds["0"]
            extruder_stack = ContainerRegistry.getInstance().findContainerStacks(id = extruder_stack_id)[0]
            return extruder_stack.getProperty(setting_key, prop)
        else:
            # Limit_to_extruder is set. The global stack handles this then
            return self._global_stack.getProperty(setting_key, prop)

    ##  Returns True if node is a descendant or the same as the root node.
    def __isDescendant(self, root: "SceneNode", node: Optional["SceneNode"]) -> bool:
        if node is None:
            return False
        if root is node:
            return True
        return self.__isDescendant(root, node.getParent())

    _affected_settings = [
        "adhesion_type", "raft_margin", "print_sequence",
        "skirt_gap", "skirt_line_count", "skirt_brim_line_width", "skirt_distance", "brim_line_count"]

    ##  Settings that change the convex hull.
    #
    #   If these settings change, the convex hull should be recalculated.
    _influencing_settings = {"xy_offset", "xy_offset_layer_0", "mold_enabled", "mold_width", "anti_overhang_mesh", "infill_mesh", "cutting_mesh"}
